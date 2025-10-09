#!/usr/bin/env python3

import rospy
import cv2
import sys
import select
import termios
import tty
import threading
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool # IMPORTED FOR PAUSE/RESUME

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/alfr3/cmd_vel", Twist, queue_size=10)
        
        # ==========================================================
        # NEW: Subscriber for pause/resume commands from controller
        # ==========================================================
        self.enable_sub = rospy.Subscriber("/enable_line_following", Bool, self.enable_callback)

        self.FORWARD_SPEED = 0.10
        self.SMOOTH_TURN_GAIN_Kp = 0.6
        self.SHARP_TURN_SPEED = 2.0

        self.twist = Twist()
        self.follow_line = False
        self.L = self.M = self.R = 0
        self.obstacle_detected = False
        self.search_direction = 1
        
        self.turning_state = False
        self.turn_direction = 0
        self.last_known_state = [0, 1, 0]

        # ==========================================================
        # NEW: State variable to track if controller has paused us
        # ==========================================================
        self.is_paused_by_controller = False
        self.lock = threading.Lock()

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

    # ==========================================================
    # NEW: Callback function to handle pause/resume messages
    # ==========================================================
    def enable_callback(self, msg):
        is_enabled = msg.data
        if is_enabled:
            rospy.loginfo("✅ LINE FOLLOWER: Resuming operation by controller.")
            self.is_paused_by_controller = False
        else:
            rospy.loginfo("⏸️ LINE FOLLOWER: Paused by controller.")
            self.is_paused_by_controller = True
            self.stop_robot() # Immediately stop when paused

    def keyboard_listener(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'a':
                        rospy.loginfo("▶️ Line following: STARTED")
                        self.follow_line = True
                    elif key == 's':
                        rospy.loginfo("⏹️ Line following: STOPPED")
                        self.follow_line = False
                        self.stop_robot()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # ==========================================================
            # MODIFIED: Added check for 'is_paused_by_controller'
            # ==========================================================
            if self.follow_line and not self.obstacle_detected and not self.is_paused_by_controller:
                with self.lock:
                    L, M, R = self.L, self.M, self.R

                if self.turning_state:
                    if M == 1:
                        self.turning_state = False
                        self.move_forward()
                    else:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = self.turn_direction * self.SHARP_TURN_SPEED
                
                else:
                    if [L, M, R] == [0, 0, 0]:
                        last_L, last_M, last_R = self.last_known_state
                        if [last_L, last_M, last_R] == [1, 0, 0]:
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = self.SHARP_TURN_SPEED
                        elif [last_L, last_M, last_R] == [0, 0, 1]:
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = -self.SHARP_TURN_SPEED
                        else:
                            self.search_line()

                    elif [L, M, R] == [1, 0, 0]:
                        self.turning_state = True
                        self.turn_direction = 1
                    elif [L, M, R] == [0, 0, 1]:
                        self.turning_state = True
                        self.turn_direction = -1
                    elif [L, M, R] == [0, 1, 0] or [L, M, R] == [1, 1, 1]:
                        self.move_forward()
                    else:
                        error = L - R
                        self.twist.linear.x = self.FORWARD_SPEED
                        self.twist.angular.z = self.SMOOTH_TURN_GAIN_Kp * error
            else:
                self.stop_robot()

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def move_forward(self):
        self.twist.linear.x = self.FORWARD_SPEED
        self.twist.angular.z = 0.0

    def search_line(self):
        self.twist.linear.x = -0.04
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1.8)
        self.stop_robot()
        rospy.sleep(0.5)
        if self.search_direction == 1:
            self.twist.angular.z = -0.7
            self.search_direction = -1
        else:
            self.twist.angular.z = 1.0
            self.search_direction = 1
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1.5)
        self.stop_robot()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
            line_detection_region = binary[int(height*0.4):, :]
            region_height, region_width = line_detection_region.shape
            third = region_width // 3
            left_roi = line_detection_region[:, :third]
            mid_roi = line_detection_region[:, third:2*third]
            right_roi = line_detection_region[:, 2*third:]
            def detect_black(roi):
                return 1 if cv2.countNonZero(roi) > 0.03 * roi.size else 0
            with self.lock:
                self.L, self.M, self.R = detect_black(left_roi), detect_black(mid_roi), detect_black(right_roi)
                current_state = [self.L, self.M, self.R]
                if current_state != [0, 0, 0]:
                    self.last_known_state = current_state
            # Obstacle detection logic removed for clarity, as it's not part of the gate task
            self.obstacle_detected = False
        except Exception as e:
            rospy.logerr("Image callback error: %s", e)

if __name__ == '__main__':
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


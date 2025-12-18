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
import random

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.follow_line = False
        self.L = self.M = self.R = 0
        self.obstacle_detected = False
        self.search_direction = 1  # 1 for right, -1 for left
        
        self.turning_state = False  # NEW: State machine variable for turns
        self.turn_direction = 0     # NEW: -1 for left, 1 for right
        self.search_count = 0

        # Proportional gain for turning, adjust this value for smoother or more aggressive turns
        self.Kp = 0.5 

        self.lock = threading.Lock()

        # Start threads
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

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
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.follow_line and not self.obstacle_detected:
                with self.lock:
                    L, M, R = self.L, self.M, self.R

                # State machine for dedicated turning
                if self.turning_state:
                    if M == 1:
                        # Exit turning state once the line is centered again
                        rospy.loginfo("✅ Line re-centered. Exiting turning state.")
                        self.turning_state = False
                        self.move_forward() # Resume forward motion
                    else:
                        # Continue turning until the line is centered
                        self.twist.linear.x = 0.0
                        # Use a dedicated turn speed based on the detected direction
                        self.twist.angular.z = self.turn_direction * 5.0
                        
                else:
                    # Normal line following logic
                    # Sharp Turn/U-Turn Logic
                    if [L, M, R] == [1, 0, 0]:
                        rospy.loginfo_throttle(1, "⬅️ Sharp Left Turn Detected! (1,0,0)")
                        self.turning_state = True
                        self.turn_direction = 1 # Turn right (positive angular.z)
                    elif [L, M, R] == [0, 0, 1]:
                        rospy.loginfo_throttle(1, "➡️ Sharp Right Turn Detected! (0,0,1)")
                        self.turning_state = True
                        self.turn_direction = -1 # Turn left (negative angular.z)
                    
                    # Minimal turns
                    elif [L, M, R] == [1, 1, 0]:
                        rospy.loginfo_throttle(1, "⬅️ Minimal Left Turn Detected! (1,1,0)")
                        self.turning_state = True
                        self.turn_direction = 1
                    elif [L, M, R] == [0, 1, 1]:
                        rospy.loginfo_throttle(1, "➡️ Minimal Right Turn Detected! (0,1,1)")
                        self.turning_state = True
                        self.turn_direction = -1
                    
                    # Intersection or Straight Line
                    elif [L, M, R] == [1, 1, 1] or [L, M, R] == [0, 1, 0]:
                        self.move_forward()
                    
                    # Lost line - Initiate search
                    elif [L, M, R] == [0, 0, 0]:
                        rospy.loginfo_throttle(1, "⚠️ Lost line. Searching...")
                        self.stop_robot()
                        self.search_line()

                    # For any other case, use proportional control
                    else:
                        error = R - L
                        angular_speed = self.Kp * error
                        self.twist.linear.x = 0.5
                        self.twist.angular.z = angular_speed
                    
            else:
                self.stop_robot()
                if self.obstacle_detected:
                    rospy.loginfo_throttle(1, "⛔ Obstacle detected! Waiting...")

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def move_forward(self):
        self.twist.linear.x = 0.10
        self.twist.angular.z = 0.0

    def search_line(self):
        """
        Makes the robot move back and then initiate a systematic search pattern.
        """
        rospy.loginfo("⏪ Moving back to find the line...")
        # Move backward at a low speed
        self.twist.linear.x = -0.05
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2.5) # Back up

        # Stop before starting the search turn
        self.stop_robot()
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.5) # Pause briefly

        # Start the turning search pattern
        if self.search_direction == 1:
            # Turn right
            self.twist.angular.z = -0.9
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.0)
            self.search_direction = -1
        else:
            # Turn left
            self.twist.angular.z = 0.9
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.0)
            self.search_direction = 1
            
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape

            # ==== LINE FOLLOWER DETECTION ====
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

            # Focus on the bottom half of the image for line detection
            line_detection_region = binary[int(height*0.5):, :]
            region_height, region_width = line_detection_region.shape

            third = region_width // 3
            left_roi = line_detection_region[:, :third]
            mid_roi = line_detection_region[:, third:2*third]
            right_roi = line_detection_region[:, 2*third:]

            def detect_black(roi):
                black_pixels = cv2.countNonZero(roi)
                total_pixels = roi.shape[0] * roi.shape[1]
                return 1 if black_pixels > 0.03 * total_pixels else 0

            with self.lock:
                self.L = detect_black(left_roi)
                self.M = detect_black(mid_roi)
                self.R = detect_black(right_roi)

            # ==== OBSTACLE DETECTION ====
            obstacle_roi = cv_image[int(height*0.6):, :]  # Bottom 40%
            hsv = cv2.cvtColor(obstacle_roi, cv2.COLOR_BGR2HSV)

            lower_obj = np.array([0, 50, 50])
            upper_obj = np.array([179, 255, 255])
            mask = cv2.inRange(hsv, lower_obj, upper_obj)

            black_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 50))
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(black_mask))

            obstacle_pixels = cv2.countNonZero(mask)
            total_pixels = mask.shape[0] * mask.shape[1]

            self.obstacle_detected = obstacle_pixels > 0.05 * total_pixels

        except Exception as e:
            rospy.logerr("Image callback error: %s", e)

if __name__ == '__main__':
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

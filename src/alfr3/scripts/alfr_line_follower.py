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

class LineFollower:
    def __init__(self):
        rospy.init_node('alfr_line_follower_node', anonymous=True)

        self.bridge = CvBridge()
        
        # ROS Topics for your alfr3 robot using ros_control
        self.image_sub = rospy.Subscriber("/alfr3/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/alfr3/diff_drive_controller/cmd_vel", Twist, queue_size=10)

        # Your original variables and logic are preserved
        self.twist = Twist()
        self.follow_line = False
        self.L = self.M = self.R = 0
        self.obstacle_detected = False
        self.search_direction = 1
        self.turning_state = False
        self.turn_direction = 0
        self.search_count = 0
        self.Kp = 0.5
        self.lock = threading.Lock()

        # Your original threading setup is preserved
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        rospy.loginfo("Line Follower Node Initialized. Press 'a' to start, 's' to stop.")

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

    # Your original control loop logic is preserved
    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.follow_line and not self.obstacle_detected:
                with self.lock:
                    L, M, R = self.L, self.M, self.R

                if self.turning_state:
                    if M == 1:
                        self.turning_state = False
                        self.move_forward()
                    else:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = self.turn_direction * 5.0
                else:
                    if [L, M, R] == [1, 0, 0]:
                        self.turning_state = True
                        self.turn_direction = 1
                    elif [L, M, R] == [0, 0, 1]:
                        self.turning_state = True
                        self.turn_direction = -1
                    elif [L, M, R] == [1, 1, 0]:
                        self.turning_state = True
                        self.turn_direction = 1
                    elif [L, M, R] == [0, 1, 1]:
                        self.turning_state = True
                        self.turn_direction = -1
                    elif [L, M, R] == [1, 1, 1] or [L, M, R] == [0, 1, 0]:
                        self.move_forward()
                    elif [L, M, R] == [0, 0, 0]:
                        self.stop_robot()
                        self.search_line()
                    else:
                        error = R - L
                        angular_speed = self.Kp * error
                        self.twist.linear.x = 0.5
                        self.twist.angular.z = angular_speed
            else:
                self.stop_robot()

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    # Your original movement logic is preserved
    def move_forward(self):
        self.twist.linear.x = 0.10
        self.twist.angular.z = 0.0

    # Your original search logic is preserved
    def search_line(self):
        self.twist.linear.x = -0.05
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2.5)
        self.stop_robot()
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.5)
        if self.search_direction == 1:
            self.twist.angular.z = -0.9
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.0)
            self.search_direction = -1
        else:
            self.twist.angular.z = 0.9
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.0)
            self.search_direction = 1
            
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape

            # === PERCEPTION LOGIC ===
            # Convert the image to grayscale, which is easier for line detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply a binary threshold. This is a critical step.
            # It turns everything darker than the threshold value (100) to white,
            # and everything lighter to black. This isolates the black line.
            # You may need to tune '100' based on the lighting in your Gazebo world.
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            # We only focus on the bottom half of the image to avoid distractions.
            line_detection_region = binary[int(height*0.5):, :]
            
            # Divide the region of interest into Left, Middle, and Right thirds.
            third = line_detection_region.shape[1] // 3
            left_roi = line_detection_region[:, :third]
            mid_roi = line_detection_region[:, third:2*third]
            right_roi = line_detection_region[:, 2*third:]

            def detect_black(roi):
                # Count the number of white pixels in the mask (which corresponds to the black line).
                black_pixels = cv2.countNonZero(roi)
                total_pixels = roi.shape[0] * roi.shape[1]
                # If the line covers more than 1% of the region, we say it's "detected".
                # This value is lowered for better sensitivity to thin lines.
                return 1 if black_pixels > 0.01 * total_pixels else 0

            # Update the global L, M, R variables safely.
            with self.lock:
                self.L = detect_black(left_roi)
                self.M = detect_black(mid_roi)
                self.R = detect_black(right_roi)

            # === VISUAL DEBUGGING ===
            # These windows are essential for tuning. They show you what the robot "sees".
            # The "Line Detection Mask" should show a clear white line on a black background.
            cv2.imshow("Robot Camera View", cv_image)
            cv2.imshow("Line Detection Mask", binary)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Image callback error: %s", e)

if __name__ == '__main__':
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        # This ensures the debug windows close cleanly when you press Ctrl+C
        cv2.destroyAllWindows()



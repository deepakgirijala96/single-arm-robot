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
        self.obstacle_detected = False  # NEW

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
            if self.follow_line and not self.obstacle_detected:  # NEW condition
                with self.lock:
                    L, M, R = self.L, self.M, self.R
                rospy.loginfo_throttle(0.5, f"📷 L:{L} M:{M} R:{R}")

                if [L, M, R] == [0, 1, 0]:
                    self.move_forward()
                elif [L, M, R] == [0, 0, 1] or [L, M, R] == [0, 1, 1]:
                    self.turn_right()
                elif [L, M, R] == [1, 0, 0] or [L, M, R] == [1, 1, 0]:
                    self.turn_left()
                elif [L, M, R] == [1, 1, 1]:
                    self.move_forward()
                else:
                    self.stop_robot()
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
        self.twist.linear.x = 0.13
        self.twist.angular.z = 0.0

    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.7

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.7

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape

            # ==== LINE FOLLOWER DETECTION ====
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

            third = width // 3
            left_roi = binary[:, :third]
            mid_roi = binary[:, third:2*third]
            right_roi = binary[:, 2*third:]

            def detect_black(roi):
                black_pixels = cv2.countNonZero(roi)
                total_pixels = roi.shape[0] * roi.shape[1]
                return 1 if black_pixels > 0.03 * total_pixels else 0  # 3% black pixels

            with self.lock:
                self.L = detect_black(left_roi)
                self.M = detect_black(mid_roi)
                self.R = detect_black(right_roi)

            # ==== OBSTACLE DETECTION ====
            # Use bottom region of the original (non-thresholded) image
            obstacle_roi = cv_image[int(height*0.6):, :]  # bottom 40%
            hsv = cv2.cvtColor(obstacle_roi, cv2.COLOR_BGR2HSV)

            # Mask for non-floor colors (excluding black line)
            # Assuming line is black and floor is bright
            lower_obj = np.array([0, 50, 50])
            upper_obj = np.array([179, 255, 255])
            mask = cv2.inRange(hsv, lower_obj, upper_obj)

            # Remove black line from mask (floor threshold)
            black_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 50))
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(black_mask))

            # Count obstacle pixels
            obstacle_pixels = cv2.countNonZero(mask)
            total_pixels = mask.shape[0] * mask.shape[1]

            # If more than 10% of bottom ROI is covered → obstacle
            self.obstacle_detected = obstacle_pixels > 0.05 * total_pixels

            # Debug visualization (optional)
            # cv2.imshow("Obstacle ROI", mask)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Image callback error: %s", e)


if __name__ == '__main__':
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


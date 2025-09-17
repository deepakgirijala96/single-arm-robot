#!/usr/bin/env python3

import rospy
import cv2
import sys, select, termios, tty, threading
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=True)

        self.bridge = CvBridge()

        # ✅ Match your URDF topics
        self.image_sub = rospy.Subscriber("/alfr3/line_camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/alfr3/diff_drive_controller/cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.follow_line = False
        self.L = self.M = self.R = 0

        self.lock = threading.Lock()

        # Keyboard listener
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        rospy.loginfo("Line Follower Node Started. Press 'a' to START, 's' to STOP.")

    def keyboard_listener(self):
        """Listen for keyboard input: a=start, s=stop"""
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'a':
                        rospy.loginfo("▶️ Line following STARTED")
                        self.follow_line = True
                    elif key == 's':
                        rospy.loginfo("⏹️ Line following STOPPED")
                        self.follow_line = False
                        self.stop_robot()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def image_callback(self, msg):
        """Process camera image and detect line segments"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

            roi = binary[int(h * 0.3):, :]  # bottom 40% of image
            third = roi.shape[1] // 3

            left = roi[:, :third]
            mid = roi[:, third:2*third]
            right = roi[:, 2*third:]

            def detect_black(region):
                black_pixels = cv2.countNonZero(region)
                total = region.shape[0] * region.shape[1]
                return 1 if black_pixels > 0.02 * total else 0

            with self.lock:
                self.L = detect_black(left)
                self.M = detect_black(mid)
                self.R = detect_black(right)

            # Process control
            self.control_logic()

        except Exception as e:
            rospy.logerr("Image processing error: %s", e)

    def control_logic(self):
        if not self.follow_line:
            return

        L, M, R = self.L, self.M, self.R

        # 🔧 Your logic
        if [L, M, R] == [1, 1, 1]:
            self.move_forward()
        elif [L, M, R] == [0, 0, 0]:
            self.stop_robot()
        elif [L, M, R] == [1, 0, 0]:
            self.turn_left(sharp=True)
        elif [L, M, R] == [0, 0, 1]:
            self.turn_right(sharp=True)
        elif [L, M, R] == [1, 1, 0]:
            self.turn_left(sharp=False)
        elif [L, M, R] == [0, 1, 1]:
            self.turn_right(sharp=False)
        else:
            self.stop_robot()

        self.cmd_vel_pub.publish(self.twist)

    def move_forward(self):
        self.twist.linear.x = 0.15
        self.twist.angular.z = 0.0

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def turn_left(self, sharp=False):
        self.twist.linear.x = 0.05 if not sharp else 0.0
        self.twist.angular.z = 1.0 if sharp else 0.5

    def turn_right(self, sharp=False):
        self.twist.linear.x = 0.05 if not sharp else 0.0
        self.twist.angular.z = -1.0 if sharp else -0.5


if __name__ == '__main__':
    try:
        node = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


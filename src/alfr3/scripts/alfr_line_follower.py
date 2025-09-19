#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sys, select, termios, tty
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import threading

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for camera
        self.image_sub = rospy.Subscriber('/line_camera/image_raw', Image, self.image_callback)

        self.bridge = CvBridge()

        # State variable
        self.active = False

        # Start keyboard listener in background
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        rospy.loginfo("Line Follower Node Started. Press 'a' to START, 's' to STOP.")

    def keyboard_listener(self):
        """Thread to capture keypress events"""
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    key = sys.stdin.read(1)
                    if key == 'a':
                        self.active = True
                        rospy.loginfo("✅ Line Following STARTED")
                    elif key == 's':
                        self.active = False
                        self.stop_robot()
                        rospy.loginfo("🛑 Line Following STOPPED")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def stop_robot(self):
        """Publish zero velocity"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if not self.active:
            return

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = frame.shape

        # Crop bottom region of interest
        crop = frame[int(0.6*height):height, 0:width]

        # Convert to grayscale + threshold for black line
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Largest contour → assume it’s the line
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                error = cx - width//2

                twist = Twist()
                twist.linear.x = 0.2
                twist.angular.z = -float(error) / 100.0

                self.cmd_vel_pub.publish(twist)

        # Debug (optional)
        # cv2.imshow("Thresh", thresh)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


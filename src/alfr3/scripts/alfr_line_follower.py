#!/usr/bin/env python3
import rospy
import cv2
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import numpy as np

class LineFollower:
    def __init__(self):
        rospy.init_node("line_follower", anonymous=True)
        
        # Camera subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)

        # Velocity publisher
        self.cmd_pub = rospy.Publisher("/diff_drive_controller/cmd_vel", Twist, queue_size=10)

        # State
        self.follow_line = False
        self.twist = Twist()
        self.rate = rospy.Rate(10)

        # Keyboard setup
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """Capture keyboard input (non-blocking)."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def image_callback(self, msg):
        if not self.follow_line:
            return  # If stopped, do nothing

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = frame.shape

        # Focus only on the bottom region of the image
        crop = frame[int(height*0.6):height, :]

        # Convert to grayscale and threshold for black
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Find contours (black regions)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Take the largest contour (the line)
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                err = cx - width // 2

                # Control law
                self.twist.linear.x = 0.1      # forward speed
                self.twist.angular.z = -float(err) / 200.0
                self.cmd_pub.publish(self.twist)
                return

        # If no line found → stop
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def run(self):
        rospy.loginfo("Press 'a' to start following, 's' to stop.")
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'a':
                rospy.loginfo("Line following started.")
                self.follow_line = True
            elif key == 's':
                rospy.loginfo("Line following stopped.")
                self.follow_line = False
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
            self.rate.sleep()

if __name__ == "__main__":
    node = LineFollower()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


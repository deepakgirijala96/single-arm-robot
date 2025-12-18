#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math

class GateDetector:
    def __init__(self):
        rospy.init_node('gate_detector_node', anonymous=True)

        # ===================================================================
        # == THE ONLY CHANGE IS HERE: The threshold is now calibrated! =====
        # ===================================================================
        self.detection_threshold = 0.4 
        # ===================================================================
        
        # State Variables
        self.gate_counter = 0
        self.is_gate_in_range = False
        self.cooldown_period = rospy.Duration(5.0)
        self.last_detection_time = rospy.Time(0)

        # Publisher & Subscriber
        self.gate_pub = rospy.Publisher('/gate_events', Int32, queue_size=10)
        rospy.Subscriber("/alfr3/front_laser", LaserScan, self.scan_callback)
        
        rospy.loginfo("✅ Gate Detector Node: Started (Calibrated).")

    def scan_callback(self, msg):
        if rospy.Time.now() - self.last_detection_time < self.cooldown_period:
            return

        # Find the minimum valid distance from all the beams
        valid_ranges = [r for r in msg.ranges if not math.isinf(r)]
        min_distance = min(valid_ranges) if valid_ranges else float('inf')

        # The core detection logic
        if min_distance < self.detection_threshold and not self.is_gate_in_range:
            self.is_gate_in_range = True
            self.gate_counter += 1
            self.gate_pub.publish(self.gate_counter)
            self.last_detection_time = rospy.Time.now()
            rospy.loginfo(f"GATE DETECTED! Number: {self.gate_counter} at distance {min_distance:.2f}m")

        elif min_distance >= self.detection_threshold and self.is_gate_in_range:
            self.is_gate_in_range = False
            rospy.loginfo("...Robot has cleared the gate's proximity.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = GateDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass



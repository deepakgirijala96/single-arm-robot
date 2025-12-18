#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32, Bool

class ALFRController:
    def __init__(self):
        rospy.init_node('alfr_controller_node', anonymous=True)

        self.enable_follower_pub = rospy.Publisher('/enable_line_following', Bool, queue_size=10)
        self.pump_pub = rospy.Publisher('/pump_control', Bool, queue_size=10)
        self.lam_led_pub = rospy.Publisher('/lam_led_control', Bool, queue_size=10)
        self.lcd_pub = rospy.Publisher('/lcd_display', String, queue_size=10)

        rospy.Subscriber('/gate_events', Int32, self.gate_callback)
        
        rospy.loginfo("✅ ALFR Controller Node: Started. Waiting for gate events.")

    def gate_callback(self, msg):
        gate_number = msg.data

        if gate_number == 1:
            rospy.loginfo("▶️ CONTROLLER: Gate 1. Activating pump.")
            self.enable_follower_pub.publish(False)
            rospy.sleep(1)
            # This now turns the pump ON and leaves it ON
            self.pump_pub.publish(True); rospy.loginfo("💧 PUMP ON")
            rospy.sleep(2) # A shorter pause
            rospy.loginfo("✅ CONTROLLER: Resuming line following.")
            self.enable_follower_pub.publish(True)

        elif gate_number == 2:
            rospy.loginfo("▶️ CONTROLLER: Gate 2. Activating LAM LEDs.")
            # This now turns the LEDs ON and leaves them ON
            self.lam_led_pub.publish(True)
            rospy.loginfo("💡 LAM LEDs ON")
            # No pause needed, the robot continues driving

        elif gate_number == 3:
            rospy.loginfo("▶️ CONTROLLER: Gate 3. Pausing to display image.")
            self.enable_follower_pub.publish(False)
            
            # Display the image on the LCD panel
            self.lcd_pub.publish("Success")
            rospy.loginfo("🖼️ Displaying Success Image!")
            rospy.sleep(5) # Pause to view the image
            
            # Resume the robot to finish the course
            rospy.loginfo("✅ CONTROLLER: Resuming for final approach.")
            self.enable_follower_pub.publish(True)
            rospy.loginfo("🏁 MISSION LOGIC COMPLETE 🏁")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ALFRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

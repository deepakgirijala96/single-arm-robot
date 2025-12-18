#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

move_speed = 0.2
turn_speed = 0.5

msg = """
Control Your SARM!
---------------------------
Moving around:
   w
a  s  d
   x

q/e : turn left/right
w/x : increase/decrease linear speed by 10%
a/d : increase/decrease angular speed by 10%

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'w': (move_speed, 0, 0),
    's': (-move_speed, 0, 0),
    'a': (0, move_speed, 0),
    'd': (0, -move_speed, 0),
    'q': (0, 0, turn_speed),
    'e': (0, 0, -turn_speed),
}

speedBindings={
    '1': (1.1, 1.0),
    '2': (0.9, 1.0),
    '3': (1.0, 1.1),
    '4': (1.0, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    # The package name doesn't actually matter for the node name
    rospy.init_node('sarm_teleop') 
    pub = rospy.Publisher('/sarm/cmd_vel', Twist, queue_size=5)

    x = 0
    y = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(f"Current speeds: linear {move_speed:.2f} m/s | angular {turn_speed:.2f} rad/s")

        while not rospy.is_shutdown():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                status = 0
            elif key in speedBindings.keys():
                move_speed = move_speed * speedBindings[key][0]
                turn_speed = turn_speed * speedBindings[key][1]
                # Update moveBindings dictionary with new speeds
                moveBindings = {
                    'w': (move_speed, 0, 0), 's': (-move_speed, 0, 0),
                    'a': (0, move_speed, 0), 'd': (0, -move_speed, 0),
                    'q': (0, 0, turn_speed), 'e': (0, 0, -turn_speed),
                }
                print(f"Current speeds: linear {move_speed:.2f} m/s | angular {turn_speed:.2f} rad/s")
                status = (status + 1) % 15
            else:
                x = 0; y = 0; th = 0
                if (key == '\x03'): break
                if key != '' and status == 0: print("Robot stopped."); status = 1

            twist = Twist()
            twist.linear.x = x; twist.linear.y = y; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

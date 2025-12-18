#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty, math

# Help message
msg = """
Control Your SARM ARM!
---------------------------
Joint Control:
 Base Rotate:   u (ccw) / j (cw)
 Shoulder:      i (up)  / k (down)
 Elbow:         o (up)  / l (down)
 Wrist:         y (up)  / h (down)
 Gripper:       g (open) / b (close)

t / v : increase/decrease step size

CTRL-C to quit
"""

# Dictionary mapping keys to joints and direction (+1 or -1)
moveBindings = {
    'u': ('base_cap', 1), 'j': ('base_cap', -1),
    'i': ('link_1', 1),   'k': ('link_1', -1),
    'o': ('link_2', 1),   'l': ('link_2', -1),
    'y': ('link_3', 1),   'h': ('link_3', -1),
    'g': ('gripper', 1),  'b': ('gripper', -1),
}

# Dictionary mapping keys to speed changes
speedBindings={
    't': 1.1, # Increase step size
    'v': 0.9, # Decrease step size
}

# Function to get a key press from the terminal
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Function to constrain angle within limits
def constrain(val, min_val, max_val):
    return max(min_val, min(max_val, val))

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('sarm_arm_teleop')

    publishers = {
        'base_cap': rospy.Publisher('/sarm/base_cap_position_controller/command', Float64, queue_size=1),
        'link_1': rospy.Publisher('/sarm/link_1_position_controller/command', Float64, queue_size=1),
        'link_2': rospy.Publisher('/sarm/link_2_position_controller/command', Float64, queue_size=1),
        'link_3': rospy.Publisher('/sarm/link_3_position_controller/command', Float64, queue_size=1),
        'gripper_1': rospy.Publisher('/sarm/gripper_1_position_controller/command', Float64, queue_size=1),
        'gripper_2': rospy.Publisher('/sarm/gripper_2_position_controller/command', Float64, queue_size=1),
    }

    target_positions = {
        'base_cap': 3.14, 'link_1': -1.5, 'link_2': 1.0,
        'link_3': 0.0, 'gripper_1': 0.0, 'gripper_2': 0.0,
    }
    
    joint_limits = {
        'base_cap': (0.0, 6.28), 'link_1': (-3.14, 0.0), 'link_2': (-0.5, 2.45),
        'link_3': (-1.5, 1.0), 'gripper_1': (0.0, 1.5), 'gripper_2': (-1.5, 0.0),
    }

    step_size = 0.1 # Radians per key press

    try:
        print(msg)
        print(f"Current step size: {step_size:.2f} radians")
        
        # Publish initial positions
        for joint, pub in publishers.items():
             if joint in target_positions:
                 pub.publish(target_positions[joint])
                 rospy.sleep(0.1)

        while not rospy.is_shutdown():
            key = getKey(settings)
            
            # Joint movement commands
            if key in moveBindings.keys():
                joint_name, direction = moveBindings[key]
                
                if joint_name == 'gripper':
                    target_positions['gripper_1'] = constrain(target_positions['gripper_1'] + direction * step_size, *joint_limits['gripper_1'])
                    target_positions['gripper_2'] = constrain(target_positions['gripper_2'] - direction * step_size, *joint_limits['gripper_2'])
                    publishers['gripper_1'].publish(target_positions['gripper_1'])
                    publishers['gripper_2'].publish(target_positions['gripper_2'])
                    # print(f"Gripper target: {target_positions['gripper_1']:.2f}") # COMMENTED OUT
                
                elif joint_name in target_positions:
                    target_positions[joint_name] = constrain(target_positions[joint_name] + direction * step_size, *joint_limits[joint_name])
                    publishers[joint_name].publish(target_positions[joint_name])
                    # print(f"{joint_name} target: {target_positions[joint_name]:.2f}") # COMMENTED OUT

            # Speed change commands
            elif key in speedBindings.keys():
                step_size *= speedBindings[key]
                print(f"Current step size: {step_size:.2f} radians")
            
            # Exit command
            elif (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        pass

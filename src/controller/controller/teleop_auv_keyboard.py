#!/usr/bin/env python3

import os
import select
import sys
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# ---- THIS PROGRAM ACTS AS INTERFACE , TAKES INPUT AND SENDS IT TO THRUSTER CONTROL FOR OUTPUT 

msg = """
Control Your AUV - 6 Pure Movements!
------------------------------------
   q
a  s  d
   x
   e

q: Pure Ascent
e: Pure Descent  
w: Pure Forward
a: Pure Left
s: Stop
d: Pure Right
x: Pure Backward

CTRL-C to quit
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_movement(description, forces):
    print(f'{description} - T1:{forces[0]:.1f}, T2:{forces[1]:.1f}, T3:{forces[2]:.1f}, T4:{forces[3]:.1f}')

def get_pure_movement(key):
    

    # ---- TWEAK THE VALUES AS NEEDED ----

    movements = {
        'q': {  # Pure Ascent 
            'description': 'Pure Ascent', 
            'forces': [0.0, 0.0, -2.0, -2.0]  
        },
        'e': {  # Pure Descent
            'description': 'Pure Descent', 
            'forces': [0.0, 0.0, 2.0, 2.0]  
        },
        'w': {  # Pure Forward
            'description': 'Pure Forward', 
            'forces': [2.0, 2.0, 0.0, 0.0]  
        },
        'a': {  # Pure Left
            'description': 'Pure Left', 
            'forces': [-10.0, 10.0, 0.0, 0.0]  
        },
        's': {  # Stop
            'description': 'Stop', 
            'forces': [0.0, 0.0, 0.0, 0.0]  
        },
        'd': {  # Pure Right
            'description': 'Pure Right', 
            'forces': [10.0, -10.0, 0.0, 0.0]  
        },
        'x': {  # Pure Backward
            'description': 'Pure Backward', 
            'forces': [-2.0, -2.0, 0.0, 0.0]  
        }
    }
    
    return movements.get(key, None)



def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    # ---- REFER TO TURTLEBOT3 TELEOP_kEYBOARD FOR BETTER UNDERSTANDING OF THE CODE ----

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_auv_pure')
    
    # Publisher for TSUP commands
    pub = node.create_publisher(Float64MultiArray, '/cmd_tsup', qos)

    # Current forces
    current_forces = [0.0, 0.0, 0.0, 0.0]

    try:

        # ---- CHANGE THE PRINT MSG AS YOU TWEAK THE VALUES----

        print(msg)
        print("CORRECTED Forces:")
        print("- Ascent/Descent: ±5.0N")
        print("- Forward/Backward: ±5.0N") 
        print("- Left/Right: 8.0N")
        while True:
            key = get_key(settings)
            
            if key == '\x03':  # CTRL + C
                break
            
            movement = get_pure_movement(key)
            
            if movement:
                current_forces = movement['forces']
                print_movement(movement['description'], current_forces)
                
                tsup_msg = Float64MultiArray()
                tsup_msg.data = current_forces
                pub.publish(tsup_msg)

    except Exception as e:
        print(e)

    finally:
        tsup_msg = Float64MultiArray()
        tsup_msg.data = [0.0, 0.0, 0.0, 0.0]
        pub.publish(tsup_msg)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import os
import select
import sys
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.qos import QoSProfile
import time

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# ---- THIS PROGRAM ACTS AS INTERFACE , TAKES INPUT AND SENDS IT TO THRUSTER CONTROL FOR OUTPUT 

msg = """
BASIC CONTROLS
------------------------------------
q  w  e
a  s  d
   x

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

def multi_keys(settings, timeout=0.3):
    keys = []
    start = time.time()  
    
    while len(keys) < 2 and (time.time() - start) < timeout:
        key = get_key(settings)
        if key == '\x03': 
            return '\x03'
        if key:
            keys.append(key)  
            if len(keys) == 1:
                start = time.time()  
    
    return ''.join(keys)  

def get_pure_movement(keys):

    # ---- TWEAK THE VALUES AS NEEDED ----

    movements = {
        'q': {'description': 'Pure Ascent', 'forces': [0.0, 0.0, -2.0, -2.0]},
        'e': {'description': 'Pure Descent', 'forces': [0.0, 0.0, 6.0, 6.0]},
        'w': {'description': 'Pure Forward', 'forces': [2.0, 2.0, 0.0, 0.0]},
        'a': {'description': 'Pure Left', 'forces': [-10.0, 10.0, 0.0, 0.0]},
        's': {'description': 'Stop', 'forces': [0.0, 0.0, 0.0, 0.0]},
        'd': {'description': 'Pure Right', 'forces': [10.0, -10.0, 0.0, 0.0]},
        'x': {'description': 'Pure Backward', 'forces': [-2.0, -2.0, 0.0, 0.0]}
    }
    
    # ---- HANDLING MULTI KEYS (STRICTLY 2 KEYS AS OF NOW) ----
    
    if not keys:
        return None
    
    if len(keys) == 1:
        return movements.get(keys[0], None)  
    
    if len(keys) == 2:
        key1, key2 = keys[0], keys[1]
        
        if 's' in keys:
            return movements['s']  
        
        move1 = movements.get(key1)  
        move2 = movements.get(key2)  
        
        if move1 and move2:
            combined_forces = [
                move1['forces'][i] + move2['forces'][i] 
                for i in range(4)
            ]
            
            return {
                'description': f"{move1['description']} + {move2['description']}",
                'forces': combined_forces
            }
    
    return None

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_auv_pure')
    
    pub = node.create_publisher(Float64MultiArray, '/cmd_tsup', qos)

    try:
        print(msg)
        print("Forces: Ascent/Descent/Forward/Backward: ±2.0N | Left/Right: ±10.0N")
        
        while True:
            keys = multi_keys(settings)  
            
            if keys == '\x03':  # CTRL + C
                break
            
            if keys: 
                movement = get_pure_movement(keys)
                if movement:
                    current_forces = movement['forces']
                    print_movement(movement['description'], current_forces)
                
                    tsup_msg = Float64MultiArray()
                    tsup_msg.data = current_forces
                    pub.publish(tsup_msg)
                else:
                    print(f"Invalid key combination: '{keys}'")

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

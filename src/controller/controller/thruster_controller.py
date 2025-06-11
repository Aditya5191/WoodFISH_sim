#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import threading

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        
        # ---- ALLOCATING THRUSTER NODES ---- 
        self.t1_ros_pub = self.create_publisher(Float64, '/thrusters/t1/thrust', 10)
        self.t2_ros_pub = self.create_publisher(Float64, '/thrusters/t2/thrust', 10)
        self.t3_ros_pub = self.create_publisher(Float64, '/thrusters/t3/thrust', 10)
        self.t4_ros_pub = self.create_publisher(Float64, '/thrusters/t4/thrust', 10)

        # ---- CURRENT VALUES FOR CONTINUOUS PUBLISHING ----
        self.current_thrusts = [0.0, 0.0, 0.0, 0.0]
        self.max_thrust = 120.0
        
        # ---- CONTINUOUS PUBLISHING TIMER (10Hz) ----
        self.publish_timer = self.create_timer(0.1, self.continuous_publish)
        
        # ---- INTERACTIVE INPUT THREAD ----
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()

        # ---- LOGS ----
        self.get_logger().info('Thrusters Initialized')
        self.get_logger().info('Enter thruster values: ')
        self.get_logger().info('Ctrl+C to exit')
        print("TSUP> ", end="", flush=True)

    def continuous_publish(self):
        
        # ---- Continuously publish current thruster values (WITH NO LOGGING) ----

        t1 = max(-self.max_thrust, min(self.max_thrust, self.current_thrusts[0]))
        t2 = max(-self.max_thrust, min(self.max_thrust, self.current_thrusts[1]))
        t3 = max(-self.max_thrust, min(self.max_thrust, self.current_thrusts[2]))
        t4 = max(-self.max_thrust, min(self.max_thrust, self.current_thrusts[3]))
        
        self.t1_ros_pub.publish(Float64(data=float(t1)))
        self.t2_ros_pub.publish(Float64(data=float(t2)))
        self.t3_ros_pub.publish(Float64(data=float(t3)))
        self.t4_ros_pub.publish(Float64(data=float(t4)))
    
    def input_handler(self):

        while rclpy.ok():
            try:
                user_input = input().strip()
                
                if user_input:
                    try:
                        values = user_input.split()
                        if len(values) == 4:
                            t1, t2, t3, t4 = map(float, values)
                            self.current_thrusts = [t1, t2, t3, t4]
                            print(f"Set: T1={t1}, T2={t2}, T3={t3}, T4={t4}")
                        else:
                            print("Enter 4 values: T1 T2 T3 T4")
                    except ValueError:
                        print("Invalid input. Use numbers only.")
                
                print("TSUP> ", end="", flush=True)
            
            except (EOFError, KeyboardInterrupt):
                break
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    controller = ThrusterController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nKilling Myself")
        controller.current_thrusts = [0.0, 0.0, 0.0, 0.0]
        controller.continuous_publish()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
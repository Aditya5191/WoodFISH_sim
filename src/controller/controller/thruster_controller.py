#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        
        # ---- ALLOCATING THRUSTER NODES ---- 

        self.t1_ros_pub = self.create_publisher(Float64, '/thrusters/t1/thrust', 10)
        self.t2_ros_pub = self.create_publisher(Float64, '/thrusters/t2/thrust', 10)
        self.t3_ros_pub = self.create_publisher(Float64, '/thrusters/t3/thrust', 10)
        self.t4_ros_pub = self.create_publisher(Float64, '/thrusters/t4/thrust', 10)

        
        # ---- TSUP ACTS AS THE ULTIMATE CONTROL THRUSTER NODE ----

        self.tsup_sub = self.create_subscription(
            Float64MultiArray,
            '/cmd_tsup',
            self.tsup_callback,
            10
        )
        
        # ---- HARDCODED PARAMETER ----

        self.max_thrust = 120.0
        
        # ---- WHEN NO MSG IS PUBLISHED ----

        self.current_thrusts = [0.0, 0.0, 0.0, 0.0]
        
        # ---- SAFETY ----

        self.last_command_time = self.get_clock().now()
        self.command_timeout = 1.0
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # ---- LOGS FOR DEBUGGING IF SOMETHING FUCKED UP ----

        self.get_logger().info('Thrusters Initializing')
        self.get_logger().info('Thruster Mapping:')
        self.get_logger().info('  - T1 → S1 (Right side thruster)')
        self.get_logger().info('  - T2 → S2 (Left side thruster)')
        self.get_logger().info('  - T3 → M2 (Rear main thruster)')
        self.get_logger().info('  - T4 → M1 (Front main thruster)')
        self.get_logger().info('Control Topic: /cmd_tsup [T1, T2, T3, T4]')
        self.get_logger().info('Publishing directly to /AUV/thrusters/tX/thrust topics')
    
    def publish_thruster_forces(self, t1_force, t2_force, t3_force, t4_force):

        # ---- CLAMPING FORCE FOR MAX FORCE ----

        t1_force = max(-self.max_thrust, min(self.max_thrust, t1_force))
        t2_force = max(-self.max_thrust, min(self.max_thrust, t2_force))
        t3_force = max(-self.max_thrust, min(self.max_thrust, t3_force))
        t4_force = max(-self.max_thrust, min(self.max_thrust, t4_force))
        
        # ---- PUBLISHES ----

        self.t1_ros_pub.publish(Float64(data=float(t1_force)))
        self.t2_ros_pub.publish(Float64(data=float(t2_force)))
        self.t3_ros_pub.publish(Float64(data=float(t3_force)))
        self.t4_ros_pub.publish(Float64(data=float(t4_force)))
        
        # ---- REAL TIME SIM ----

        self.current_thrusts = [t1_force, t2_force, t3_force, t4_force]
        self.last_command_time = self.get_clock().now()
        
        # ---- LOGGING ----

        self.get_logger().info(
            f'T1: {t1_force:.1f}N, T2: {t2_force:.1f}N, '
            f'T3: {t3_force:.1f}N, T4: {t4_force:.1f}N'
        )
    
    def tsup_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().error(f'TSUP needs 4 values, got {len(msg.data)}')
            return
        
        self.publish_thruster_forces(msg.data[0], msg.data[1], msg.data[2], msg.data[3])
    
    def safety_check(self):
        
        # ---- STOP IFF NO COMMANDS RECIEVED ----

        time_since_last = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if time_since_last > self.command_timeout and any(t != 0.0 for t in self.current_thrusts):
            self.publish_thruster_forces(0.0, 0.0, 0.0, 0.0)
            self.get_logger().warn('Command timeout - stopping thrusters')

def main(args=None):
    rclpy.init(args=args)
    controller = ThrusterController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down gracefully, I suppose')
        controller.publish_thruster_forces(0.0, 0.0, 0.0, 0.0)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
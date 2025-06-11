#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Wrench, Twist
import numpy as np
import math

# Import Gazebo Transport for publishing gz.msgs.Double
try:
    import gz.transport13 as gz_transport
    from gz.msgs10.double_pb2 import Double
    GZ_TRANSPORT_VERSION = 13
except ImportError:
    try:
        import gz.transport12 as gz_transport
        from gz.msgs9.double_pb2 import Double
        GZ_TRANSPORT_VERSION = 12
    except ImportError:
        print("Error: Gazebo Transport Python bindings not found")
        exit(1)

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        
        # Initialize Gazebo Transport node
        self.gz_node = gz_transport.Node()
        
        # Gazebo Transport publishers for individual thrusters
        # Based on your mapping: S1=T1, S2=T2, M2=T3, M1=T4
        self.t1_topic = "/AUV/thrusters/t1/thrust"  # S1 - Right side thruster
        self.t2_topic = "/AUV/thrusters/t2/thrust"  # S2 - Left side thruster
        self.t3_topic = "/AUV/thrusters/t3/thrust"  # M2 - Rear main thruster
        self.t4_topic = "/AUV/thrusters/t4/thrust"  # M1 - Front main thruster
        
        # FIXED: Create publishers properly
        try:
            # Method 1: Try with message class
            self.t1_pub = self.gz_node.advertise(self.t1_topic, Double)
            self.t2_pub = self.gz_node.advertise(self.t2_topic, Double)
            self.t3_pub = self.gz_node.advertise(self.t3_topic, Double)
            self.t4_pub = self.gz_node.advertise(self.t4_topic, Double)
            self.get_logger().info("Gazebo Transport publishers created successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to create Gazebo Transport publishers: {e}")
            # Fallback: Don't use Gazebo Transport, rely on bridge only
            self.t1_pub = None
            self.t2_pub = None
            self.t3_pub = None
            self.t4_pub = None
        
        # ROS2 Publishers for bridge compatibility
        self.t1_ros_pub = self.create_publisher(Float64, '/thrusters/t1/thrust', 10)
        self.t2_ros_pub = self.create_publisher(Float64, '/thrusters/t2/thrust', 10)
        self.t3_ros_pub = self.create_publisher(Float64, '/thrusters/t3/thrust', 10)
        self.t4_ros_pub = self.create_publisher(Float64, '/thrusters/t4/thrust', 10)
        
        # ROS2 Subscribers for different control modes
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        self.wrench_sub = self.create_subscription(
            Wrench, 
            '/cmd_wrench', 
            self.wrench_callback, 
            10
        )
        
        # NEW: Combined thruster command subscriber
        self.tsup_sub = self.create_subscription(
            Float64MultiArray,
            '/cmd_tsup',
            self.tsup_callback,
            10
        )
        
        # Individual thruster command subscribers for direct control
        self.t1_sub = self.create_subscription(Float64, '/cmd_t1', self.t1_callback, 10)
        self.t2_sub = self.create_subscription(Float64, '/cmd_t2', self.t2_callback, 10)
        self.t3_sub = self.create_subscription(Float64, '/cmd_t3', self.t3_callback, 10)
        self.t4_sub = self.create_subscription(Float64, '/cmd_t4', self.t4_callback, 10)
        
        # AUV Physical Parameters (calculated from your XACRO)
        # Center of mass: (0.004, 0, 0.033) from base_link
        # Main thrusters: M1 at (0.125, 0, 0.07), M2 at (-0.125, 0, 0.07)
        # Side thrusters: S1 at (0.005, 0.085, 0.035), S2 at (0.005, -0.085, 0.035)
        
        # Moment arms for torque calculations
        self.yaw_moment_arm = 0.125      # Distance from center to main thrusters (M1, M2)
        self.roll_moment_arm = 0.085     # Distance from center to side thrusters (S1, S2)
        self.pitch_moment_arm = 0.035    # Vertical offset for pitch control
        
        # Thruster separations
        self.main_thruster_separation = 0.25   # Total separation between M1 and M2
        self.side_thruster_separation = 0.17   # Total separation between S1 and S2
        
        # Control parameters
        self.max_thrust = 50.0           # Maximum thrust per thruster (Newtons)
        self.thrust_coefficient = 1.0    # From your Gazebo plugin config
        self.fluid_density = 1000.0      # Water density (kg/m³)
        
        # Scaling factors for different control modes
        self.linear_scale = 20.0         # Scale factor for linear velocities
        self.angular_scale = 10.0        # Scale factor for angular velocities
        self.force_scale = 1.0           # Scale factor for wrench forces
        self.torque_scale = 1.0          # Scale factor for wrench torques
        
        # Current thruster values
        self.current_thrusts = [0.0, 0.0, 0.0, 0.0]  # [T1, T2, T3, T4]
        
        # Control mode tracking
        self.last_command_time = self.get_clock().now()
        self.command_timeout = 1.0  # seconds
        
        # Create timer for safety monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('=== Thruster Controller Node Started ===')
        self.get_logger().info(f'Gazebo Transport Version: {GZ_TRANSPORT_VERSION}')
        self.get_logger().info('AUV Configuration:')
        self.get_logger().info(f'  - Yaw moment arm: {self.yaw_moment_arm:.3f} m')
        self.get_logger().info(f'  - Roll moment arm: {self.roll_moment_arm:.3f} m')
        self.get_logger().info(f'  - Max thrust per thruster: {self.max_thrust:.1f} N')
        self.get_logger().info('Thruster Mapping:')
        self.get_logger().info('  - T1 → S1 (Right side thruster)')
        self.get_logger().info('  - T2 → S2 (Left side thruster)')
        self.get_logger().info('  - T3 → M2 (Rear main thruster)')
        self.get_logger().info('  - T4 → M1 (Front main thruster)')
        self.get_logger().info('Control Topics:')
        self.get_logger().info('  - /cmd_tsup: Combined thruster control [T1, T2, T3, T4]')
        self.get_logger().info('  - /cmd_vel: Twist velocity commands')
        self.get_logger().info('  - /cmd_wrench: Force/torque commands')
        self.get_logger().info('Publishing to both Gazebo Transport and ROS2 topics')
    
    def publish_thruster_forces(self, t1_force, t2_force, t3_force, t4_force):
        """Publish individual thruster forces to both Gazebo Transport and ROS2"""
        # Clamp forces to maximum thrust
        t1_force = max(-self.max_thrust, min(self.max_thrust, t1_force))
        t2_force = max(-self.max_thrust, min(self.max_thrust, t2_force))
        t3_force = max(-self.max_thrust, min(self.max_thrust, t3_force))
        t4_force = max(-self.max_thrust, min(self.max_thrust, t4_force))
        
        # Publish to ROS2 topics (for bridge)
        t1_ros_msg = Float64()
        t1_ros_msg.data = float(t1_force)
        self.t1_ros_pub.publish(t1_ros_msg)
        
        t2_ros_msg = Float64()
        t2_ros_msg.data = float(t2_force)
        self.t2_ros_pub.publish(t2_ros_msg)
        
        t3_ros_msg = Float64()
        t3_ros_msg.data = float(t3_force)
        self.t3_ros_pub.publish(t3_ros_msg)
        
        t4_ros_msg = Float64()
        t4_ros_msg.data = float(t4_force)
        self.t4_ros_pub.publish(t4_ros_msg)
        
        # FIXED: Try to publish to Gazebo Transport if available
        if self.t1_pub is not None:
            try:
                # Create gz.msgs.Double messages
                t1_msg = Double()
                t1_msg.data = float(t1_force)
                
                t2_msg = Double()
                t2_msg.data = float(t2_force)
                
                t3_msg = Double()
                t3_msg.data = float(t3_force)
                
                t4_msg = Double()
                t4_msg.data = float(t4_force)
                
                # Use the publisher objects to publish
                self.t1_pub.publish(t1_msg)
                self.t2_pub.publish(t2_msg)
                self.t3_pub.publish(t3_msg)
                self.t4_pub.publish(t4_msg)
                
            except Exception as e:
                self.get_logger().warn(f"Failed to publish to Gazebo Transport: {e}")
        
        # Update current thrust values and timestamp
        self.current_thrusts = [t1_force, t2_force, t3_force, t4_force]
        self.last_command_time = self.get_clock().now()
        
        # Log current thruster values
        self.get_logger().info(
            f'Thrusts - T1(S1): {t1_force:.2f}N, T2(S2): {t2_force:.2f}N, '
            f'T3(M2): {t3_force:.2f}N, T4(M1): {t4_force:.2f}N'
        )
    
    def tsup_callback(self, msg):
        """NEW: Combined thruster command - accepts [T1, T2, T3, T4] values"""
        if len(msg.data) != 4:
            self.get_logger().error(
                f'tsup command must have exactly 4 values [T1, T2, T3, T4], '
                f'received {len(msg.data)} values'
            )
            return
        
        # Extract thruster forces from array
        t1_force = msg.data[0]  # T1 = S1 (Right side)
        t2_force = msg.data[1]  # T2 = S2 (Left side)
        t3_force = msg.data[2]  # T3 = M2 (Rear main)
        t4_force = msg.data[3]  # T4 = M1 (Front main)
        
        # Publish the forces
        self.publish_thruster_forces(t1_force, t2_force, t3_force, t4_force)
        
        self.get_logger().info(
            f'RECEIVED TSUP Command - T1: {t1_force:.2f}N, T2: {t2_force:.2f}N, '
            f'T3: {t3_force:.2f}N, T4: {t4_force:.2f}N'
        )
    
    def cmd_vel_callback(self, msg):
        """Convert Twist commands to thruster forces"""
        # Extract desired velocities
        linear_x = msg.linear.x    # Forward/backward
        linear_y = msg.linear.y    # Left/right (strafe)
        linear_z = msg.linear.z    # Up/down (not used in this config)
        angular_x = msg.angular.x  # Roll (not used in this config)
        angular_y = msg.angular.y  # Pitch (not used in this config)
        angular_z = msg.angular.z  # Yaw rotation
        
        # Scale velocities to thrust forces
        forward_thrust = linear_x * self.linear_scale
        lateral_thrust = linear_y * self.linear_scale
        yaw_thrust = angular_z * self.angular_scale
        
        # Thruster allocation based on your AUV geometry
        # T1 = S1 (Right side), T2 = S2 (Left side), T3 = M2 (Rear), T4 = M1 (Front)
        
        # Forward/backward motion: Main thrusters T3 (rear) and T4 (front)
        # Positive forward_thrust moves AUV forward
        t3_forward = forward_thrust  # Rear thruster pushes forward
        t4_forward = forward_thrust  # Front thruster pushes forward
        
        # Lateral motion: Side thrusters T1 (right) and T2 (left)
        # Positive lateral_thrust moves AUV to the right
        t1_lateral = lateral_thrust   # Right thruster pushes right
        t2_lateral = -lateral_thrust  # Left thruster pushes left (opposite)
        
        # Yaw rotation: Differential thrust on main thrusters
        # Positive yaw_thrust rotates AUV counter-clockwise (left turn)
        yaw_differential = yaw_thrust / 2.0
        t3_yaw = -yaw_differential  # Rear thruster contributes to yaw
        t4_yaw = yaw_differential   # Front thruster contributes to yaw
        
        # Combine motions
        t1_force = t1_lateral                    # T1 = S1 (Right side)
        t2_force = t2_lateral                    # T2 = S2 (Left side)
        t3_force = t3_forward + t3_yaw          # T3 = M2 (Rear main)
        t4_force = t4_forward + t4_yaw          # T4 = M1 (Front main)
        
        self.publish_thruster_forces(t1_force, t2_force, t3_force, t4_force)
        
        self.get_logger().debug(
            f'Cmd_vel: lin=({linear_x:.2f},{linear_y:.2f}), ang_z={angular_z:.2f}'
        )
    
    def wrench_callback(self, msg):
        """Convert wrench commands to thruster forces using thruster allocation matrix"""
        # Extract desired forces and torques
        fx = msg.force.x * self.force_scale    # Forward force
        fy = msg.force.y * self.force_scale    # Lateral force
        fz = msg.force.z * self.force_scale    # Vertical force (not used)
        tx = msg.torque.x * self.torque_scale  # Roll torque (not used)
        ty = msg.torque.y * self.torque_scale  # Pitch torque (not used)
        tz = msg.torque.z * self.torque_scale  # Yaw torque
        
        # Thruster allocation matrix based on your AUV geometry
        # Rows: [Fx, Fy, Fz, Tx, Ty, Tz]
        # Cols: [T1(S1), T2(S2), T3(M2), T4(M1)]
        
        # T1=S1: Right side thruster at (0.005, 0.085, 0.035)
        # T2=S2: Left side thruster at (0.005, -0.085, 0.035)  
        # T3=M2: Rear main thruster at (-0.125, 0, 0.07)
        # T4=M1: Front main thruster at (0.125, 0, 0.07)
        
        allocation_matrix = np.array([
            # Fx: Forward force (main thrusters contribute)
            [0.0, 0.0, 1.0, 1.0],
            # Fy: Lateral force (side thrusters contribute)  
            [1.0, -1.0, 0.0, 0.0],
            # Fz: Vertical force (none in this configuration)
            [0.0, 0.0, 0.0, 0.0],
            # Tx: Roll torque (side thrusters with vertical offset)
            [self.pitch_moment_arm, -self.pitch_moment_arm, 0.0, 0.0],
            # Ty: Pitch torque (main thrusters with vertical offset)
            [0.0, 0.0, -self.pitch_moment_arm, self.pitch_moment_arm],
            # Tz: Yaw torque (differential thrust on main thrusters)
            [0.0, 0.0, -self.yaw_moment_arm, self.yaw_moment_arm]
        ])
        
        # Desired wrench vector
        wrench_vector = np.array([fx, fy, fz, tx, ty, tz])
        
        # Solve for thruster forces using pseudo-inverse
        try:
            thruster_forces = np.linalg.pinv(allocation_matrix) @ wrench_vector
            
            # Extract individual thruster forces
            t1_force = thruster_forces[0]  # T1 = S1
            t2_force = thruster_forces[1]  # T2 = S2
            t3_force = thruster_forces[2]  # T3 = M2
            t4_force = thruster_forces[3]  # T4 = M1
            
            self.publish_thruster_forces(t1_force, t2_force, t3_force, t4_force)
            
            self.get_logger().debug(
                f'Wrench: F=({fx:.2f},{fy:.2f},{fz:.2f}), T=({tx:.2f},{ty:.2f},{tz:.2f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Thruster allocation failed: {e}')
            self.stop_all_thrusters()
    
    # Individual thruster command callbacks for direct control
    def t1_callback(self, msg):
        """Direct T1 (S1) control"""
        self.current_thrusts[0] = msg.data
        self.publish_thruster_forces(
            self.current_thrusts[0], 
            self.current_thrusts[1], 
            self.current_thrusts[2], 
            self.current_thrusts[3]
        )
        self.get_logger().debug(f'Direct T1 command: {msg.data:.2f}N')
    
    def t2_callback(self, msg):
        """Direct T2 (S2) control"""
        self.current_thrusts[1] = msg.data
        self.publish_thruster_forces(
            self.current_thrusts[0], 
            self.current_thrusts[1], 
            self.current_thrusts[2], 
            self.current_thrusts[3]
        )
        self.get_logger().debug(f'Direct T2 command: {msg.data:.2f}N')
    
    def t3_callback(self, msg):
        """Direct T3 (M2) control"""
        self.current_thrusts[2] = msg.data
        self.publish_thruster_forces(
            self.current_thrusts[0], 
            self.current_thrusts[1], 
            self.current_thrusts[2], 
            self.current_thrusts[3]
        )
        self.get_logger().debug(f'Direct T3 command: {msg.data:.2f}N')
    
    def t4_callback(self, msg):
        """Direct T4 (M1) control"""
        self.current_thrusts[3] = msg.data
        self.publish_thruster_forces(
            self.current_thrusts[0], 
            self.current_thrusts[1], 
            self.current_thrusts[2], 
            self.current_thrusts[3]
        )
        self.get_logger().debug(f'Direct T4 command: {msg.data:.2f}N')
    
    def safety_check(self):
        """Safety monitoring - stop thrusters if no commands received"""
        current_time = self.get_clock().now()
        time_since_last_command = (current_time - self.last_command_time).nanoseconds / 1e9
        
        if time_since_last_command > self.command_timeout:
            # Stop all thrusters if no commands received for timeout period
            if any(thrust != 0.0 for thrust in self.current_thrusts):
                self.stop_all_thrusters()
                self.get_logger().warn('Command timeout - stopping all thrusters')
    
    def stop_all_thrusters(self):
        """Emergency stop - set all thrusters to zero"""
        self.publish_thruster_forces(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('All thrusters stopped')
    
    def get_thruster_status(self):
        """Return current thruster status"""
        return {
            'T1_S1': self.current_thrusts[0],
            'T2_S2': self.current_thrusts[1], 
            'T3_M2': self.current_thrusts[2],
            'T4_M1': self.current_thrusts[3],
            'total_thrust': sum(abs(t) for t in self.current_thrusts)
        }

def main(args=None):
    rclpy.init(args=args)
    controller = ThrusterController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down thruster controller')
        controller.stop_all_thrusters()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

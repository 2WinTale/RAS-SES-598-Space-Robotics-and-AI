#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import signal
import sys

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],    
            [-1/(self.M * self.L)]
        ])
        
        # LQR cost matrices
        self.Q = np.diag([20.0, 10.0, 50.0, 30.0])  # State cost
        self.R = np.array([[0.05]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        
        # Data logging for performance graph
        self.time_steps = []
        self.state_log = []
        self.control_log = []
        self.time = 0.0
        
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force', 
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info('Cart-Pole LQR Controller initialized')
        
        # Handle shutdown signal
        signal.signal(signal.SIGINT, self.handle_shutdown)
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')
            
            self.x = np.array([
                [msg.position[cart_idx]],
                [msg.velocity[cart_idx]],
                [msg.position[pole_idx]],
                [msg.velocity[pole_idx]]
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: {self.x.T}')
                self.state_initialized = True
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}')
    
    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if not self.state_initialized:
                return

            # Compute control input u = -Kx
            u = -self.K @ self.x
            force = float(u[0])
            
            # Log data for plotting
            self.time_steps.append(self.time)
            self.state_log.append(self.x.flatten())
            self.control_log.append(force)
            self.time += 0.01
            
            # Publish control command
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
    
    def plot_performance(self):
        """Plot LQR performance graph and keep it open."""
        if len(self.time_steps) == 0:
            return
        
        self.state_log = np.array(self.state_log)
        
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(self.time_steps, self.state_log[:, 0], label='Cart Position (x)')
        plt.plot(self.time_steps, self.state_log[:, 1], label='Cart Velocity (ẋ)')
        plt.plot(self.time_steps, self.state_log[:, 2], label='Pole Angle (θ)')
        plt.plot(self.time_steps, self.state_log[:, 3], label='Pole Angular Velocity (θ̇)')
        plt.xlabel('Time (s)')
        plt.ylabel('State Values')
        plt.legend()
        plt.title('LQR Performance - State Evolution')
        
        plt.subplot(2, 1, 2)
        plt.plot(self.time_steps, self.control_log, label='Control Force (N)', color='r')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.title('LQR Control Force Over Time')
        
        plt.tight_layout()
        plt.pause(0.1)  # Small pause for interactive mode
        plt.show(block=True)  # Keeps the plot open until closed manually
    
    def handle_shutdown(self, signum, frame):
        """Handle shutdown signal and generate plot."""
        self.get_logger().info('Generating LQR performance graph...')
        self.plot_performance()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.handle_shutdown(signal.SIGINT, None)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

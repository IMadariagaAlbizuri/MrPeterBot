#!/usr/bin/env python3

# Mr. PeterBot LQR controller Algorithm

# Import Libraries
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.linalg import solve_continuous_are  # Solves the continuous-time algebraic Riccati equation
import math

# Message structures for the topics
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# Create the LQR Controller Class
class LQRController(Node):

    # Constructor
    def __init__(self):
        super().__init__('lqr_controller')  # Node with name 'lqr_controller'

        # Parameters Declaration
        self.l       = 0.26     # wheelbase [m]
        self.tau_v   = 0.1      # velocity time constant [s]
        self.v_max   = 0.2      # max velocity [m/s]
        self.phi_max = 0.6108   # max steering angle [rad]
        self.rate    = 100.0    # control rate [Hz]

        # LQR Controller parameters
        # Q penalises state error
        self.Q = np.diag([1.0, 5.0]) 

        # R penalises control effort  
        self.R = np.diag([5.0, 5.0]) 

        # System internal state during control
        self.v0     = 0.0
        self.theta0 = 0.0
        self.phi0   = 0.0

        # References
        self.v_ref     = 0.0
        self.theta_ref = 0.0

        # Topics subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.v_ref_sub = self.create_subscription(
            Float64,
            '/v_ref',
            self.v_ref_callback,
            10
        )

        self.theta_ref_sub = self.create_subscription(
            Float64,
            '/theta_ref',
            self.theta_ref_callback,
            10
        )

        # Publishers
        self.velocity_pub = self.create_publisher(
            Float64,
            '/velocity',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64,
            '/steering_angle',
            10
        )

        # For fixed rate control loop
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.control_loop
        )

        # INFO when Node Starts
        self.get_logger().info('LQR Controller initialised — waiting for odometry...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        # Update linear velocity
        self.v0 = msg.twist.twist.linear.x

        # Update the orientation
        q = msg.pose.pose.orientation  # Quaternions
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta0 = math.atan2(siny_cosp, cosy_cosp)

    def joint_states_callback(self, msg: JointState):
        # Update Steering angle
        # This case from left steering joint
        if 'front_left_steering_joint' in msg.name:
            idx = msg.name.index('front_left_steering_joint')
            self.phi0 = msg.position[idx]

    def v_ref_callback(self, msg: Float64):
        # Update reference velocity
        self.v_ref = max(-self.v_max, min(self.v_max, msg.data))

    def theta_ref_callback(self, msg: Float64):
        # Update reference heading
        self.theta_ref = msg.data

    # ── LQR Logic ─────────────────────────────────────────────────────────────

    # 1º - Get A and B matrices. Depend on the operation point of the system
    def compute_AB(self):
        # Equations obtained from the Linearisation expression
        # See README.md

        # Minimum velocity for numerical stability of Riccati
        v0 = self.v0 if abs(self.v0) > 0.01 else 0.01

        A = np.array([
            [-1.0 / self.tau_v,            0.0],
            [math.tan(self.phi0) / self.l, 0.0]
        ])

        cos_phi  = math.cos(self.phi0)
        sec2_phi = 1.0 / (cos_phi ** 2) if abs(cos_phi) > 1e-6 else 1e6  # To avoid division by 0

        B = np.array([
            [1.0 / self.tau_v,                        0.0],
            [0.0,                  v0 * sec2_phi / self.l]
        ])

        return A, B

    # 2º - Get the optimal gain K
    def compute_lqr_gain(self, A, B):
        # Solve Riccati Equation
        try:
            P = solve_continuous_are(A, B, self.Q, self.R)
            K = np.linalg.inv(self.R) @ B.T @ P
            return K
        except Exception as e:
            self.get_logger().warn(f'Riccati solver failed: {e}')
            return None

    # 3º - Define control loop
    def control_loop(self):
        # Skip LQR when no velocity reference is set
        if abs(self.v_ref) < 0.01:
            return

        # 1. Build A, B at current operating point
        A, B = self.compute_AB()

        # 2. Solve Riccati → get K
        K = self.compute_lqr_gain(A, B)
        if K is None:
            return

        # 3. Compute tracking error: δx = x_ref - x₀
        delta_v     = self.v_ref     - self.v0
        delta_theta = self.theta0 - self.theta_ref

        # Wrap heading error to [-π, π]
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
        delta_x = np.array([delta_v, delta_theta])

        # 4. Apply control law: δu = K · δx
        delta_u = K @ delta_x

        # Actuator signal values
        v_cmd   = float(np.clip(delta_u[0], -self.v_max,   self.v_max))
        phi_cmd = float(np.clip(delta_u[1], -self.phi_max, self.phi_max))

        # Publish actuator values
        v_msg   = Float64()
        phi_msg = Float64()

        v_msg.data   = v_cmd
        phi_msg.data = phi_cmd

        self.velocity_pub.publish(v_msg)
        self.steering_pub.publish(phi_msg)

        self.get_logger().debug(
            f'v₀={self.v0:.2f} θ₀={self.theta0:.2f} φ₀={self.phi0:.2f} | '
            f'v_ref={self.v_ref:.2f} θ_ref={self.theta_ref:.2f} | '
            f'v_cmd={v_cmd:.2f} φ_cmd={phi_cmd:.2f}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
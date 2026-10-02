#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetEntityState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def euler_to_quaternion(roll, pitch, yaw):
    """Transform Euler Angles (rad) to quaternion (qw, qx, qy, qz)."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

class StingGuidanceController(Node):
    def __init__(self):
        super().__init__('sting_guidance_controller')

        # QoS Profile Compatible with Sensors/Estimators in C++. No frame loss
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # EKF Topic Subscription
        self.ekf_sub = self.create_subscription(
            PoseStamped, 
            '/target/estimated_pose', 
            self.ekf_callback, 
            qos_profile
        )

        # Interception Signal Publisher
        self.hit_pub = self.create_publisher(Bool, '/target/intercepted', 10)

        # Gazebo State Service Client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Service /gazebo/set_entity_state...')

        # Kinematic State Sting
        self.pos_sting = np.array([0.0, 0.0, 0.1])
        self.vel_sting = np.array([0.0, 0.0, 0.0])

        # Objective Estimated State
        self.pos_target_est = None
        self.vel_target_est = np.array([0.0, 0.0, 0.0])
        self.last_target_pos = None

        # Interceptor and PNG Guidance Parameters
        self.max_speed = 26.0      
        self.max_accel = 90.0      
        self.kill_distance = 0.5   
        self.nav_constant = 4.5    

        # Falling State Config
        self.is_falling = False
        self.fall_speed = 0.0
        self.spin_angle = 0.0

        self.alpha_vel = 0.70 #EMA to Estimate Objetive Position with Position Differnetial
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.guidance_loop)
        self.get_logger().info('Sting-II PNG Guidance Active. Listening to /target/estimated_pose...')

    def ekf_callback(self, msg: PoseStamped):
        current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # --- Compute Objective Velocity with Derivative EKF Position between Samples ---
        if self.last_target_pos is not None:
            raw_vel = (current_pos - self.last_target_pos) / self.dt
            self.vel_target_est = self.alpha_vel * raw_vel + (1.0 - self.alpha_vel) * self.vel_target_est

        self.pos_target_est = current_pos
        self.last_target_pos = current_pos

    def check_continuous_collision(self, p_start, p_end, p_target, radius):
        """Prevent Tunneling (Object is so fast that Collsion may Occur between Frames)."""
        segment = p_end - p_start
        seg_len_sq = np.dot(segment, segment)

        if seg_len_sq < 1e-6:
            return np.linalg.norm(p_start - p_target) <= radius

        # Project Target Pos onto Motion Segment [0, 1]
        t = max(0.0, min(1.0, np.dot(p_target - p_start, segment) / seg_len_sq))
        closest_point = p_start + t * segment
        return np.linalg.norm(closest_point - p_target) <= radius

    def guidance_loop(self):
        if self.pos_target_est is None:
            self.get_logger().warn('Waiting Data from /target/estimated_pose...', throttle_duration_sec=2.0)
            return

        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        if not self.is_falling:
            prev_pos_sting = np.copy(self.pos_sting)

            # 1. Line of Sight Vector (LOS): Relative Vector towards Objective and its Unitary Vector
            r_los = self.pos_target_est - self.pos_sting
            r_mag = np.linalg.norm(r_los)
            u_los = r_los / (r_mag + 1e-6)

            # 2. Relative Velocity Vector
            v_rel = self.vel_target_est - self.vel_sting
            v_closing = -np.dot(u_los, v_rel)   # How fast Distance is Closing

            speed_sting = np.linalg.norm(self.vel_sting)

            if speed_sting < 5.0:
                a_cmd = u_los * self.max_accel
            else:
                # 3. Angular Velocity of LOS (Omega)
                omega_los = np.cross(r_los, v_rel) / (r_mag ** 2 + 1e-6)

                # 4. Proportional Guidance Acceleration (PNG 3D)
                effective_v_closing = max(v_closing, 10.0)
                a_pn = self.nav_constant * effective_v_closing * np.cross(omega_los, u_los)
                a_pursuit = 2.0 * (u_los * self.max_speed - self.vel_sting) # Added to ensure Convergence Velocity Vector w Objective Trajectory
                a_cmd = a_pn + a_pursuit

            accel_mag = np.linalg.norm(a_cmd)
            if accel_mag > self.max_accel:
                a_cmd = (a_cmd / accel_mag) * self.max_accel

            # 5. Kinematic Integration
            self.vel_sting += a_cmd * self.dt # Integarte Accel -> Vel
            speed = np.linalg.norm(self.vel_sting)
            if speed > self.max_speed:
                self.vel_sting = (self.vel_sting / speed) * self.max_speed

            self.pos_sting += self.vel_sting * self.dt  # Integrate Vel -> Pos

            # 6. Continuous Collision Detection (CCD) Check
            if self.check_continuous_collision(prev_pos_sting, self.pos_sting, self.pos_target_est, self.kill_distance):
                self.get_logger().info('*** PHYSICAL INTERCEPTION DETECTED! ***')
                self.is_falling = True
                hit_msg = Bool()
                hit_msg.data = True
                self.hit_pub.publish(hit_msg)
                return

            # 7. Visual Orientation Alignment
            if speed > 0.1:
                v_xy = np.sqrt(self.vel_sting[0]**2 + self.vel_sting[1]**2)
                
                # Apply -90 deg (-pi/2) offset on Yaw to align mesh Forward Vector w Vel
                yaw = np.arctan2(self.vel_sting[1], self.vel_sting[0]) - (np.pi / 2.0)
                pitch = np.arctan2(-self.vel_sting[2], v_xy)
                roll = 0.0

                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        else:
            # Post-Collision Tumbling and Free-Fall Physics
            self.fall_speed += 9.81 * self.dt
            self.pos_sting[2] -= self.fall_speed * self.dt
            if self.pos_sting[2] <= 0.1:
                self.pos_sting[2] = 0.1

            self.spin_angle += 0.3
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.5, self.spin_angle)

        # Transmit Updated State to Gazebo Simulator
        state_req = SetEntityState.Request()
        state_req.state.name = 'sting2'
        state_req.state.reference_frame = 'world'
        state_req.state.pose.position.x = float(self.pos_sting[0])
        state_req.state.pose.position.y = float(self.pos_sting[1])
        state_req.state.pose.position.z = float(self.pos_sting[2])
        state_req.state.pose.orientation.x = float(qx)
        state_req.state.pose.orientation.y = float(qy)
        state_req.state.pose.orientation.z = float(qz)
        state_req.state.pose.orientation.w = float(qw)

        self.set_state_client.call_async(state_req)

def main(args=None):
    rclpy.init(args=args)
    node = StingGuidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
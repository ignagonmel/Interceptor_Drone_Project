#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetEntityState

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

    return qw, qx, qy, qz

class ShahedFlightController(Node):
    def __init__(self):
        super().__init__('shahed_flight_controller')

        # Publishers matching ROS 2 architecture
        self.pose_pub = self.create_publisher(PoseStamped, '/target/true_pose', 10)
        self.meas_pub = self.create_publisher(Point, '/target/measurement', 10)
        self.path_pub = self.create_publisher(Path, '/target/true_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/target/marker', 10)

        # Subscriber for interception signal from Sting guidance. Signals if interception occurred
        self.hit_sub = self.create_subscription(
            Bool, '/target/intercepted', self.hit_callback, 10)

        # Gazebo state service client
        self.client_ = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        # Flight State
        self.t = 0.0
        self.noise_std = 0.35
        self.is_hit = False
        self.fall_speed = 0.0
        self.spin_angle = 0.0

        # Position State. First State (20, 0, 25)
        self.x = 20.0
        self.y = 0.0
        self.z = 25.0

        # Path history
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Timer loop at 50 Hz = 0.02s
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update_flight)

        self.get_logger().info('Shahed-136 flight controller active.')

    def hit_callback(self, msg: Bool): 
        if msg.data and not self.is_hit:
            self.is_hit = True
            self.get_logger().info('Shahed-136 HIT! Commencing crash sequence...')

    def update_flight(self):
        self.t += self.dt

        if not self.is_hit:
            # 1. Operational Position. Equations to Define Drone Movement
            self.x = 15.0 * np.cos(0.4 * self.t) + 5.0 * np.sin(0.8 * self.t)
            self.y = 15.0 * np.sin(0.4 * self.t)
            self.z = 25.0 + 3.0 * np.sin(0.6 * self.t)

            # 2. Velocity Vector. Derivative of Position Equations in 1.
            vx = -6.0 * np.sin(0.4 * self.t) + 4.0 * np.cos(0.8 * self.t)
            vy = 6.0 * np.cos(0.4 * self.t)
            vz = 1.8 * np.cos(0.6 * self.t)

            # 3. Orientation Regarding Movement
            v_xy = np.sqrt(vx**2 + vy**2)
            yaw = np.arctan2(vy, vx) + np.pi
            pitch = np.arctan2(vz, v_xy)  # Pitching (Ascent/Descent)
            roll = 0.0

            qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
        else:
            # Falling after Impact
            self.fall_speed += 9.81 * self.dt
            self.z -= self.fall_speed * self.dt
            if self.z <= 0.1:
                self.z = 0.1

            self.spin_angle += 0.2
            qw, qx, qy, qz = euler_to_quaternion(0.0, 0.5, self.spin_angle)

        now = self.get_clock().now().to_msg()

        # 1. Publish Ground Truth Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(self.x)
        pose_msg.pose.position.y = float(self.y)
        pose_msg.pose.position.z = float(self.z)
        pose_msg.pose.orientation.w = float(qw)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        self.pose_pub.publish(pose_msg)

        # 2. Publish Path
        self.path_msg.header.stamp = now
        self.path_msg.poses.append(pose_msg)
        if len(self.path_msg.poses) > 500:  # Maintain a Clean Trajectory History
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

        # 3. Publish RViz Marker
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'map'
        marker.ns = 'shahed'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.6, 0.6, 0.6
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.8
        self.marker_pub.publish(marker)

        # 4. Publish Noisy Measurement for C++ EKF. True Position + N(0, noise)
        if not self.is_hit:
            meas_msg = Point()
            meas_msg.x = float(self.x + np.random.normal(0, self.noise_std))
            meas_msg.y = float(self.y + np.random.normal(0, self.noise_std))
            meas_msg.z = float(self.z + np.random.normal(0, self.noise_std))
            self.meas_pub.publish(meas_msg)

        # 5. Update Gazebo Entity
        state_req = SetEntityState.Request()
        state_req.state.name = 'shahed136'
        state_req.state.reference_frame = 'world'
        state_req.state.pose.position.x = float(self.x)
        state_req.state.pose.position.y = float(self.y)
        state_req.state.pose.position.z = float(self.z)
        state_req.state.pose.orientation.w = float(qw)
        state_req.state.pose.orientation.x = float(qx)
        state_req.state.pose.orientation.y = float(qy)
        state_req.state.pose.orientation.z = float(qz)
        self.client_.call_async(state_req)

def main(args=None):
    rclpy.init(args=args)
    node = ShahedFlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
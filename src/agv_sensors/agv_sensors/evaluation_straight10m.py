#!/usr/bin/env python3
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


def quat_from_yaw(yaw_rad: float):
    """Return quaternion (x,y,z,w) for yaw around Z."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavGoalAndLogger(Node):
    def __init__(self):
        super().__init__("nav_goal_and_logger")

        # ===== Parameters =====
        self.declare_parameter("out", "pose_log.csv")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "Base_footprint")

        # Goal default: x=10, y=0, z=0, yaw=0deg
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 0.0)
        self.declare_parameter("goal_yaw_deg", 0.0)

        self.out_path = self.get_parameter("out").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate").get_parameter_value().double_value)
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter("child_frame").get_parameter_value().string_value

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_z = float(self.get_parameter("goal_z").value)
        self.goal_yaw_deg = float(self.get_parameter("goal_yaw_deg").value)

        # ===== TF listener =====
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # ===== CSV =====
        self.f = open(self.out_path, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["t_ros", "x", "y", "yaw_deg"])
        self.f.flush()

        # ===== Nav2 Action Client =====
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Logging control
        self.logging_enabled = False

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self.log_tick)

        # Start workflow: wait action server -> send goal
        self.startup_timer = self.create_timer(0.2, self.start_once)
        self.started = False

        self.get_logger().info(
            f"Will send Nav2 goal (x={self.goal_x}, y={self.goal_y}, z={self.goal_z}, yaw={self.goal_yaw_deg}deg). "
            f"Then log TF {self.parent_frame}->{self.child_frame} to {self.out_path} @ {self.rate_hz}Hz"
        )

    def start_once(self):
        if self.started:
            return
        self.started = True
        self.startup_timer.cancel()

        self.get_logger().info("Waiting for Nav2 action server: /navigate_to_pose ...")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available. Is Nav2 running?")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.parent_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = self.goal_z

        yaw_rad = math.radians(self.goal_yaw_deg)
        qx, qy, qz, qw = quat_from_yaw(yaw_rad)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info("Sending goal...")
        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted. Start logging to CSV now.")
        self.logging_enabled = True

        # (Optional) If you want to stop logging when goal finishes, keep this:
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        # You can print feedback if needed; leave quiet to avoid spam.
        pass

    def result_cb(self, future):
        # Nav2 finished (success/failed/canceled)
        try:
            status = future.result().status
            self.get_logger().info(f"Navigation finished. status={status}. Logging continues (press Ctrl+C to stop).")
        except Exception as e:
            self.get_logger().warn(f"Result callback error: {e}")

    def log_tick(self):
        if not self.logging_enabled:
            return
        try:
            tf = self.buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())

            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            x = tf.transform.translation.x
            y = tf.transform.translation.y

            q = tf.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            yaw_deg = math.degrees(yaw)

            self.writer.writerow([f"{t:.6f}", f"{x:.6f}", f"{y:.6f}", f"{yaw_deg:.3f}"])
            self.f.flush()

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = NavGoalAndLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

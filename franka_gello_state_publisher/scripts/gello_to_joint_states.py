#!/usr/bin/env python3
"""Visualize the GELLO arm(s) on the dual FR3 in RViz, without the RT host.

Subscribes to whichever of /left/gello/joint_states and /right/gello/joint_states are
actually advertised (names fr3_joint1..7) plus the matching GELLO gripper width
(0 closed .. 1 open), and publishes the full prismatic_fr3_duo /joint_states: every arm
with a live GELLO follows it, every other joint stays at its value from
prismatic_fr3_duo_moveit_config/config/initial_positions.yaml.

Topics are rediscovered while running, so starting a second GELLO later is picked up
automatically; set the `sides` parameter to pin it to one arm instead.

Do not run this while the RT host is up: it would publish a second /joint_states.

Usage (inside the container, after sourcing scripts/ros_env.sh):
    python3 /ros2_ws/src/franka_gello_state_publisher/scripts/gello_to_joint_states.py
    python3 .../gello_to_joint_states.py --ros-args -p sides:="['left']"
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

SIDES = ("left", "right")
FINGER_MAX_POSITION = 0.04  # m, URDF upper limit of fr3_finger_joint1
PUBLISH_RATE = 50.0  # Hz
DISCOVERY_PERIOD = 2.0  # s, how often to look for GELLO topics that are not up yet


def gello_topic(side):
    return f"/{side}/gello/joint_states"


def gripper_topic(side):
    return f"/{side}/gripper/gripper_client/target_gripper_width_percent"


class GelloToJointStates(Node):
    def __init__(self, node_name="gello_to_joint_states", output_topic="/joint_states"):
        super().__init__(node_name)
        self.declare_parameter("sides", list(SIDES))
        self._sides = [
            side
            for side in self.get_parameter("sides").get_parameter_value().string_array_value
            if side in SIDES
        ]

        initial_positions_path = os.path.join(
            get_package_share_directory("prismatic_fr3_duo_moveit_config"),
            "config",
            "initial_positions.yaml",
        )
        with open(initial_positions_path, encoding="utf-8") as f:
            initial_positions = yaml.safe_load(f)["initial_positions"]
        self._positions = {name: float(value) for name, value in initial_positions.items()}

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self._output_topic = output_topic
        self._subscriptions_by_topic = {}
        # Publish at a fixed rate so MoveIt has a current state before GELLO data arrives.
        self.create_timer(1.0 / PUBLISH_RATE, self._publish)
        self.create_timer(DISCOVERY_PERIOD, self._discover)
        self._discover()
        if not self._subscriptions_by_topic:
            self.get_logger().info(
                f"No GELLO topics yet for {self._sides}; waiting (publishing initial positions)"
            )

    def _discover(self):
        """Subscribe to the GELLO topics of every side that is advertised but not yet wired."""
        available = {name for name, _ in self.get_topic_names_and_types()}
        for side in self._sides:
            joints, gripper = gello_topic(side), gripper_topic(side)
            if joints in available and joints not in self._subscriptions_by_topic:
                self._subscriptions_by_topic[joints] = self.create_subscription(
                    JointState, joints, self._make_gello_callback(side), 10
                )
                self.get_logger().info(f"Mapping {joints} -> {self._output_topic} ({side}_fr3_*)")
            if gripper in available and gripper not in self._subscriptions_by_topic:
                self._subscriptions_by_topic[gripper] = self.create_subscription(
                    Float32, gripper, self._make_gripper_callback(side), 10
                )
                self.get_logger().info(
                    f"Mapping {gripper} -> {self._output_topic} ({side}_fr3_finger_joint1)"
                )

    def _make_gello_callback(self, side):
        prefix = side + "_"

        def on_gello(msg):
            for name, position in zip(msg.name, msg.position):
                target = prefix + name
                if target in self._positions:
                    self._positions[target] = position

        return on_gello

    def _make_gripper_callback(self, side):
        # finger_joint2 mimics finger_joint1 in the URDF
        finger_joint = f"{side}_fr3_finger_joint1"

        def on_gripper(msg):
            self._positions[finger_joint] = msg.data * FINGER_MAX_POSITION

        return on_gripper

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._positions)
        msg.position = list(self._positions.values())
        self._publisher.publish(msg)


def main(**node_kwargs):
    rclpy.init()
    node = GelloToJointStates(**node_kwargs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

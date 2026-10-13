#!/usr/bin/env python3
"""Drive the Isaac Sim prismatic_fr3_duo from the available GELLO arm(s).

Same mapping as gello_to_joint_states.py, published to /isaac_joint_commands as joint
position targets: each arm with a live GELLO (left, right, or both) follows it, every
other joint holds
its value from prismatic_fr3_duo_moveit_config/config/initial_positions.yaml.

Do not run this while ros2_control is running with hardware_mode isaac: its
topic_based_ros2_control hardware also publishes /isaac_joint_commands.

Usage (inside the container, after sourcing scripts/ros_env.sh):
    python3 /ros2_ws/src/franka_gello_state_publisher/scripts/gello_to_isaac_joint_commands.py
"""
from gello_to_joint_states import main

if __name__ == "__main__":
    # main(node_name="gello_to_isaac_joint_commands", output_topic="/isaac_joint_commands")
    main(node_name="gello_to_isaac_joint_commands", output_topic="/dual_arm_controller_wbc/target_joint")


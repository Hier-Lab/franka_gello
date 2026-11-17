// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_fr3_arm_controllers/joint_impedance_controller.hpp>

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

using std::placeholders::_1;

namespace franka_fr3_arm_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates_();
  // Freshness is re-evaluated every cycle: the subscription callback cannot notice a publisher
  // that stopped publishing.
  readGelloSample_();

  Vector7d q_goal;
  Vector7d tau_d_calculated;

  if (!hold_position_initialized_) {
    // Latch the measured configuration on the first cycle after activation.
    q_hold_ = q_;
    hold_position_initialized_ = true;
  }

  if (!motion_generator_initialized_) {
    // After starting the controller we wait for fresh joint states from the input topic.
    // Until then we hold the configuration the robot had at activation.
    motion_generator_initialized_ = initializeMotionGenerator_();

    if (!motion_generator_initialized_) {
      tau_d_calculated = calculateTauDGains_(q_hold_);
      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau_d_calculated(i));
      }

      return controller_interface::return_type::OK;
    }
  }

  if (!move_to_start_position_finished_) {
    // We have received valid joint states and initialized the motion generator
    // Now we move smoothly to the first joint position received from the input topic
    auto trajectory_time = this->get_node()->now() - start_time_;
    auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
    move_to_start_position_finished_ = motion_generator_output.second;

    q_goal = motion_generator_output.first;
  }

  if (move_to_start_position_finished_) {
    // After reaching the start position we follow the joint position from the input topic
    // This is the normal operation mode of the controller
    if (!gello_position_values_valid_) {
      RCLCPP_FATAL(get_node()->get_logger(), "Timeout: No valid joint states received from Gello");
      rclcpp::shutdown();  // Exit the node permanently
    }
    for (int i = 0; i < num_joints; ++i) {
      q_goal(i) = gello_position_values_[i];
    }
  }

  tau_d_calculated = calculateTauDGains_(q_goal);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

void JointImpedanceController::jointStateCallback_(const sensor_msgs::msg::JointState msg) {
  // Anything that arrives while the controller is not active is dropped, so a message from an
  // earlier session can never become the start target of a later activation.
  if (!accepting_gello_messages_.load()) {
    return;
  }

  GelloSample sample;
  if (msg.position.size() < sample.positions.size()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received joint state size is smaller than expected size.");
    return;
  }

  std::copy(msg.position.begin(), msg.position.begin() + sample.positions.size(),
            sample.positions.begin());
  sample.stamp = rclcpp::Time(msg.header.stamp, RCL_ROS_TIME);
  sample.receive_time = get_node()->now();
  sample.seq = ++gello_rx_seq_;
  gello_buffer_.writeFromNonRT(sample);
}

void JointImpedanceController::readGelloSample_() {
  const GelloSample& sample = *gello_buffer_.readFromRT();

  if (sample.seq == 0) {
    // Nothing received since activation.
    gello_position_values_valid_ = false;
    consecutive_fresh_gello_samples_ = 0;
    return;
  }

  const auto now = get_node()->now();
  const double receive_age = (now - sample.receive_time).seconds();
  const double stamp_age = (now - sample.stamp).seconds();
  const bool fresh = sample.receive_time >= activation_time_ && receive_age < kMaxJointStateAge &&
                     stamp_age < kMaxJointStateAge && stamp_age > -kMaxJointStateAge;

  if (!fresh) {
    if (gello_position_values_valid_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Gello joint states are stale. Time since received: %f // Time since "
                           "message stamp: %f",
                           receive_age, stamp_age);
    }
    gello_position_values_valid_ = false;
    consecutive_fresh_gello_samples_ = 0;
    return;
  }

  if (sample.seq != last_seen_gello_seq_) {
    last_seen_gello_seq_ = sample.seq;
    if (consecutive_fresh_gello_samples_ < kRequiredFreshGelloSamples) {
      ++consecutive_fresh_gello_samples_;
    }
  }
  gello_position_values_ = sample.positions;
  gello_position_values_valid_ = true;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  namespace_prefix_ = get_node()->get_namespace();
  if (namespace_prefix_ == "/" || namespace_prefix_.empty()) {
    namespace_prefix_.clear();
  } else {
    // Remove leading slash and add trailing underscore
    namespace_prefix_ = namespace_prefix_.substr(1) + "_";
  }

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto k_alpha = get_node()->get_parameter("k_alpha").as_double();

  if (!validateGains_(k_gains, "k_gains") || !validateGains_(d_gains, "d_gains")) {
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  if (k_alpha < 0.0 || k_alpha > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_alpha should be in the range [0, 1]");
    return CallbackReturn::FAILURE;
  }

  k_alpha_ = k_alpha;

  dq_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  accepting_gello_messages_.store(false);
  joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "gello/joint_states", 1,
      [this](const sensor_msgs::msg::JointState& msg) { jointStateCallback_(msg); });

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Discard whatever was buffered by a previous activation before accepting new messages.
  accepting_gello_messages_.store(false);
  gello_buffer_.writeFromNonRT(GelloSample{});
  activation_time_ = get_node()->now();
  last_seen_gello_seq_ = 0;
  consecutive_fresh_gello_samples_ = 0;
  gello_position_values_valid_ = false;
  hold_position_initialized_ = false;
  motion_generator_initialized_ = false;
  move_to_start_position_finished_ = false;
  dq_filtered_.setZero();
  start_time_ = this->get_node()->now();
  accepting_gello_messages_.store(true);

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated. Holding current position until %s publishes fresh joint states.",
              joint_state_subscriber_->get_topic_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  accepting_gello_messages_.store(false);
  gello_buffer_.writeFromNonRT(GelloSample{});
  return CallbackReturn::SUCCESS;
}

auto JointImpedanceController::calculateTauDGains_(const Vector7d& q_goal) -> Vector7d {
  dq_filtered_ = (1 - k_alpha_) * dq_filtered_ + k_alpha_ * dq_;
  Vector7d tau_d_calculated;
  tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);

  return tau_d_calculated;
}

bool JointImpedanceController::validateGains_(const std::vector<double>& gains,
                                              const std::string& gains_name) {
  if (gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s parameter not set", gains_name.c_str());
    return false;
  }

  if (gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s should be of size %d but is of size %ld",
                 gains_name.c_str(), num_joints, gains.size());
    return false;
  }

  return true;
}

void JointImpedanceController::updateJointStates_() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool JointImpedanceController::initializeMotionGenerator_() {
  // Only start once the topic has delivered an unbroken run of fresh messages after this
  // activation; a single stray or old message is not enough.
  if (!gello_position_values_valid_ ||
      consecutive_fresh_gello_samples_ < kRequiredFreshGelloSamples) {
    // Only send a warning once every 10 seconds in order not to spam the log
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 10 * 1000,
                         "Waiting for fresh joint states on %s ...",
                         joint_state_subscriber_->get_topic_name());
    return false;
  }

  Vector7d q_goal;
  updateJointStates_();
  for (int i = 0; i < num_joints; ++i) {
    q_goal(i) = gello_position_values_[i];
  }
  RCLCPP_INFO(get_node()->get_logger(), "q_goal of motion generator: [%f, %f, %f, %f, %f, %f, %f]",
              q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5), q_goal(6));

  const double motion_generator_speed_factor = 0.2;
  motion_generator_ = std::make_unique<MotionGenerator>(motion_generator_speed_factor, q_, q_goal);
  return true;
}

}  // namespace franka_fr3_arm_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_fr3_arm_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)

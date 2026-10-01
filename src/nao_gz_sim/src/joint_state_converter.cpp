// Copyright 2026
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

/// Task 3B joint-state converter.
///
/// Subscribes to the internal raw JointState produced by ros_gz_bridge from the
/// Gazebo joint-state publisher, validates it against the frozen Task 3A
/// contract and republishes the single public /joint_states topic with the
/// frozen QoS. The stock bridge fills effort[] with a vector of zeros and is
/// therefore never exposed; the converter always publishes effort as empty.
///
/// This node is read-only with respect to the simulation: it never publishes
/// joint commands, never touches a controller and never writes to the model.

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
/// Frozen canonical joint order (canonical SDF document order, Task 3A §3.5).
const std::vector<std::string> kExpectedJoints = {
  "HeadYaw", "HeadPitch",
  "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
  "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
  "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
  "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
};
}  // namespace

class JointStateConverter : public rclcpp::Node
{
public:
  JointStateConverter()
  : rclcpp::Node("joint_state_converter")
  {
    const std::string raw_topic =
      this->declare_parameter<std::string>("raw_topic", "/nao/joint_states_raw");
    const std::string output_topic =
      this->declare_parameter<std::string>("output_topic", "/joint_states");
    const int qos_depth = this->declare_parameter<int>("qos_depth", 10);

    rclcpp::QoS in_qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth))));
    in_qos.reliable().durability_volatile();
    rclcpp::QoS out_qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth))));
    out_qos.reliable().durability_volatile();

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic, out_qos);
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      raw_topic, in_qos,
      std::bind(&JointStateConverter::on_raw, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joint_state_converter: raw='%s' -> output='%s' (reliable, volatile, keep_last %d); "
      "effort is forced empty", raw_topic.c_str(), output_topic.c_str(), qos_depth);
  }

private:
  void fail(const std::string & reason)
  {
    if (!failed_) {
      failed_ = true;
      RCLCPP_ERROR(this->get_logger(), "CONTRACT VIOLATION: %s; publishing stopped",
                   reason.c_str());
    }
  }

  void on_raw(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (failed_) {
      return;
    }

    // Skip the not-yet-started case (no valid simulation time).
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u) {
      return;
    }

    if (msg->name.size() != kExpectedJoints.size()) {
      fail("expected " + std::to_string(kExpectedJoints.size()) + " joint names, got " +
           std::to_string(msg->name.size()));
      return;
    }
    if (msg->position.size() != kExpectedJoints.size()) {
      fail("position length " + std::to_string(msg->position.size()) + " != " +
           std::to_string(kExpectedJoints.size()));
      return;
    }
    if (msg->velocity.size() != kExpectedJoints.size()) {
      fail("velocity length " + std::to_string(msg->velocity.size()) + " != " +
           std::to_string(kExpectedJoints.size()));
      return;
    }
    for (size_t i = 0; i < kExpectedJoints.size(); ++i) {
      if (msg->name[i] != kExpectedJoints[i]) {
        fail("joint name/order mismatch at index " + std::to_string(i) + ": got '" +
             msg->name[i] + "', expected '" + kExpectedJoints[i] + "'");
        return;
      }
    }

    // Timestamp policy: strictly increasing Gazebo simulation time.
    const auto & stamp = msg->header.stamp;
    if (have_last_stamp_) {
      const bool backwards =
        (stamp.sec < last_stamp_.sec) ||
        (stamp.sec == last_stamp_.sec && stamp.nanosec < last_stamp_.nanosec);
      const bool duplicate =
        (stamp.sec == last_stamp_.sec && stamp.nanosec == last_stamp_.nanosec);
      if (backwards) {
        fail("simulation timestamp went backwards: " +
             std::to_string(last_stamp_.sec) + "." + std::to_string(last_stamp_.nanosec) +
             " -> " + std::to_string(stamp.sec) + "." + std::to_string(stamp.nanosec));
        return;
      }
      if (duplicate) {
        ++duplicate_stamps_;
        return;  // do not publish a non-increasing duplicate
      }
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "";
    out.name = msg->name;
    out.position = msg->position;
    out.velocity = msg->velocity;
    out.effort.clear();  // never forward the stock bridge's zero effort vector

    pub_->publish(out);
    last_stamp_ = stamp;
    have_last_stamp_ = true;
    ++published_;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  builtin_interfaces::msg::Time last_stamp_{};
  bool have_last_stamp_{false};
  bool failed_{false};
  uint64_t published_{0};
  uint64_t duplicate_stamps_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateConverter>());
  rclcpp::shutdown();
  return 0;
}

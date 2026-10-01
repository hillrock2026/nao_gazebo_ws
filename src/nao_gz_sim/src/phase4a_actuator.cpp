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

/// Task 4 Phase 4A actuator (gz-sim system plugin).
///
/// This plugin is the *execution* half of the Phase 4A closed loop. It is
/// loaded into the runtime ``nao`` model and is the only component that writes
/// joint commands into the Gazebo simulation. It subscribes to the frozen
/// command topic (``/joint_command``, ``sensor_msgs/msg/JointState``,
/// 24-dimensional physical position target) using rclcpp inside the Gazebo
/// server process and applies a bounded P position servo through the standard
/// ``gz::sim::components::JointVelocityCmd`` interface, which the Gazebo
/// physics system consumes every step.
///
/// Contract handling (frozen schema v1.0.0 section 2.3/5.4/6):
///   * exactly 24 names and 24 positions, in the frozen joint order;
///   * ``velocity`` and ``effort`` must be empty;
///   * ``header.frame_id`` must be empty;
///   * ``header.stamp`` must be non-zero, finite and strictly increasing;
///   * every position finite and within the SDF joint limit.
/// Any violation is rejected, counted and logged; the previous accepted target
/// is kept. A command is never clamped or silently corrected.
///
/// Only a valid command ever enables actuation. Before the first valid command
/// the plugin writes nothing, so a missing probe cannot fake execution.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <sdf/Element.hh>
#include <sdf/JointAxis.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace nao_gz_sim
{
namespace
{
/// Frozen canonical joint order (Task 3A / Task4 schema section 4).
const std::vector<std::string> kJointOrder = {
  "HeadYaw", "HeadPitch",
  "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
  "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
  "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
  "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
};
}  // namespace

class NaoPhase4aActuator
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  NaoPhase4aActuator() = default;

  ~NaoPhase4aActuator() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    sub_.reset();
    node_.reset();
    executor_.reset();
    if (we_inited_rclcpp_) {
      rclcpp::shutdown();
    }
  }

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    topic_ = _sdf->Get<std::string>("joint_command_topic", "/joint_command").first;
    p_gain_ = _sdf->Get<double>("p_gain", 8.0).first;
    max_velocity_ = _sdf->Get<double>("max_velocity", 2.0).first;
    // Safe idle hold (Phase 4B, default off to preserve the verified Phase 4A
    // behaviour). When true, until the first valid command arrives the actuator
    // holds whatever joint positions exist at start, so the model does not
    // collapse during DDS discovery of the producer.
    hold_on_start_ = _sdf->Get<bool>("hold_on_start", false).first;

    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR(
        this->RosLoggerSafe(),
        "nao_phase4a_actuator: plugin must be attached to a model entity");
      failed_ = true;
      return;
    }

    joints_.resize(kJointOrder.size());
    for (size_t i = 0; i < kJointOrder.size(); ++i) {
      JointBinding & b = joints_[i];
      b.name = kJointOrder[i];
      b.entity = model.JointByName(_ecm, b.name);
      if (b.entity == gz::sim::kNullEntity) {
        RCLCPP_ERROR(this->RosLoggerSafe(),
                     "nao_phase4a_actuator: joint '%s' not found in model", b.name.c_str());
        failed_ = true;
        continue;
      }
      const auto axis = _ecm.Component<gz::sim::components::JointAxis>(b.entity);
      if (axis) {
        b.has_limits = true;
        b.lower = axis->Data().Lower();
        b.upper = axis->Data().Upper();
      } else {
        b.has_limits = false;
      }
    }
    if (failed_) {
      return;
    }

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      we_inited_rclcpp_ = true;
    }
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->RosLoggerSafe(),
                   "nao_phase4a_actuator: rclcpp not available; plugin disabled");
      failed_ = true;
      return;
    }

    node_ = std::make_shared<rclcpp::Node>("nao_phase4a_actuator");
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable().durability_volatile();
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_, qos,
      std::bind(&NaoPhase4aActuator::OnCommand, this, std::placeholders::_1));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    RCLCPP_INFO(
      node_->get_logger(),
      "nao_phase4a_actuator ready: topic='%s' joints=%zu p_gain=%.2f max_velocity=%.2f",
      topic_.c_str(), joints_.size(), p_gain_, max_velocity_);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (failed_ || _info.paused) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_target_) {
      if (hold_on_start_ && !hold_initialized_) {
        // Capture the current (start) joint positions once and hold them until
        // the first valid command arrives.
        std::vector<double> captured;
        captured.reserve(joints_.size());
        bool complete = true;
        for (const JointBinding & b : joints_) {
          const auto pos = _ecm.Component<gz::sim::components::JointPosition>(b.entity);
          if (!pos || pos->Data().empty()) {
            complete = false;
            break;
          }
          captured.push_back(pos->Data()[0]);
        }
        if (complete) {
          target_ = captured;
          have_target_ = true;
          hold_initialized_ = true;
          RCLCPP_INFO(this->RosLoggerSafe(),
                      "nao_phase4a_actuator: hold_on_start captured start pose");
        }
      }
      if (!have_target_) {
        return;
      }
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      const JointBinding & b = joints_[i];
      const auto pos = _ecm.Component<gz::sim::components::JointPosition>(b.entity);
      if (!pos || pos->Data().empty()) {
        continue;
      }
      const double current = pos->Data()[0];
      const double error = target_[i] - current;
      const double velocity = std::clamp(p_gain_ * error, -max_velocity_, max_velocity_);

      auto vel_cmd =
        _ecm.Component<gz::sim::components::JointVelocityCmd>(b.entity);
      if (vel_cmd == nullptr) {
        _ecm.CreateComponent(
          b.entity, gz::sim::components::JointVelocityCmd(std::vector<double>{velocity}));
      } else {
        vel_cmd->Data().assign(1, velocity);
      }
    }
  }

private:
  struct JointBinding
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    bool has_limits{false};
    double lower{0.0};
    double upper{0.0};
  };

  /// Logger usable before the rclcpp node exists (Configure error paths).
  rclcpp::Logger RosLoggerSafe()
  {
    if (node_) {
      return node_->get_logger();
    }
    return rclcpp::get_logger("nao_phase4a_actuator");
  }

  void Reject(const std::string & reason)
  {
    const int count = ++reject_counts_[reason];
    if (count == 1 || count % 100 == 0) {
      RCLCPP_WARN(this->RosLoggerSafe(),
                  "nao_phase4a_actuator: rejected command (%s), count=%d",
                  reason.c_str(), count);
    }
  }

  void OnCommand(const sensor_msgs::msg::JointState::SharedPtr _msg)
  {
    if (failed_) {
      return;
    }
    if (_msg->name.size() != kJointOrder.size() ||
      _msg->position.size() != kJointOrder.size())
    {
      Reject("dim");
      return;
    }
    for (size_t i = 0; i < kJointOrder.size(); ++i) {
      if (_msg->name[i] != kJointOrder[i]) {
        Reject("order");
        return;
      }
    }
    if (!_msg->velocity.empty()) {
      Reject("velocity_not_empty");
      return;
    }
    if (!_msg->effort.empty()) {
      Reject("effort_not_empty");
      return;
    }
    if (!_msg->header.frame_id.empty()) {
      Reject("frame_id_not_empty");
      return;
    }
    const auto & stamp = _msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0u) {
      Reject("zero_stamp");
      return;
    }
    if (stamp.sec < 0) {
      Reject("negative_stamp");
      return;
    }
    for (double value : _msg->position) {
      if (!std::isfinite(value)) {
        Reject("non_finite");
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < joints_.size(); ++i) {
      const JointBinding & b = joints_[i];
      if (!b.has_limits) {
        continue;
      }
      const double value = _msg->position[i];
      if (value < b.lower - 1e-9 || value > b.upper + 1e-9) {
        Reject("out_of_limit");
        return;
      }
    }

    if (have_target_) {
      const bool duplicate =
        stamp.sec == last_stamp_.sec && stamp.nanosec == last_stamp_.nanosec;
      const bool backwards =
        stamp.sec < last_stamp_.sec ||
        (stamp.sec == last_stamp_.sec && stamp.nanosec < last_stamp_.nanosec);
      if (duplicate) {
        Reject("duplicate_stamp");
        return;
      }
      if (backwards) {
        Reject("backward_stamp");
        return;
      }
    }

    target_.assign(_msg->position.begin(), _msg->position.end());
    last_stamp_ = stamp;
    if (!have_target_) {
      RCLCPP_INFO(this->RosLoggerSafe(),
                  "nao_phase4a_actuator: first valid command accepted (stamp %d.%09u)",
                  stamp.sec, stamp.nanosec);
    }
    have_target_ = true;
    ++accepted_;
  }

  std::string topic_;
  double p_gain_{8.0};
  double max_velocity_{2.0};
  bool hold_on_start_{false};
  bool hold_initialized_{false};
  // Written during Configure (Gazebo thread) and read by the physics thread
  // (PreUpdate) and the rclcpp executor thread (OnCommand); atomic to avoid a
  // strict-memory-model data race (Phase 4A review P3 #1).
  std::atomic<bool> failed_{false};

  std::vector<JointBinding> joints_;
  std::vector<double> target_;
  bool have_target_{false};
  builtin_interfaces::msg::Time last_stamp_{};
  uint64_t accepted_{0};
  std::mutex mutex_;
  std::map<std::string, int> reject_counts_;

  bool we_inited_rclcpp_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};
}  // namespace nao_gz_sim

GZ_ADD_PLUGIN(
  nao_gz_sim::NaoPhase4aActuator,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  nao_gz_sim::NaoPhase4aActuator,
  "nao_gz_sim::NaoPhase4aActuator")

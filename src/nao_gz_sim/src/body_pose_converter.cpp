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

/// Task 3C body-pose converter.
///
/// Subscribes directly to the Gazebo topic publishing ``gz.msgs.Pose_V``
/// (default ``/world/nao_world/pose/info``) through gz-transport and publishes
/// the frozen ``/body_pose`` (geometry_msgs/msg/PoseStamped) for the canonical
/// body frame ``base_link`` (== the spawned model ``nao``).
///
/// The Pose_V -> ROS bridge is deliberately NOT used: it drops the entity names
/// and the header stamp (verified in Task 3C investigation), so it cannot select
/// the model by name nor preserve the simulation timestamp.
///
/// This node never publishes /tf or /tf_static, never uses a ROS timer to
/// generate poses and never rewrites the Gazebo timestamp.

#include <cmath>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class BodyPoseConverter : public rclcpp::Node
{
public:
  BodyPoseConverter()
  : rclcpp::Node("body_pose_converter")
  {
    model_name_ = this->declare_parameter<std::string>("model_name", "nao");
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/world/nao_world/pose/info");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/body_pose");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable().durability_volatile();
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, qos);

    if (!gz_node_.Subscribe(pose_topic_, &BodyPoseConverter::on_pose, this)) {
      RCLCPP_ERROR(this->get_logger(),
                   "failed to subscribe to Gazebo topic '%s'", pose_topic_.c_str());
      failed_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "body_pose_converter: gz '%s' model '%s' -> '%s' (frame_id=world)",
                  pose_topic_.c_str(), model_name_.c_str(), output_topic_.c_str());
    }
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

  void on_pose(const gz::msgs::Pose_V & msg)
  {
    if (failed_) {
      return;
    }

    const auto & stamp = msg.header().stamp();
    if (stamp.sec() == 0 && stamp.nsec() == 0) {
      return;  // no valid simulation time yet
    }

    const gz::msgs::Pose * found = nullptr;
    int matches = 0;
    for (const auto & pose : msg.pose()) {
      if (pose.name() == model_name_) {
        found = &pose;
        ++matches;
      }
    }
    if (matches == 0) {
      // Before the model has ever been seen this is the normal pre-spawn phase;
      // afterwards a missing model is a violation (deleted/reset).
      if (have_seen_model_) {
        fail("model '" + model_name_ + "' disappeared from " + pose_topic_);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "model '%s' not present in %s yet; waiting",
                             model_name_.c_str(), pose_topic_.c_str());
      }
      return;
    }
    if (matches > 1) {
      fail("ambiguous model name '" + model_name_ + "' in " + pose_topic_ +
           ", found " + std::to_string(matches));
      return;
    }
    have_seen_model_ = true;

    const auto & p = found->position();
    const auto & q = found->orientation();
    const double px = p.x(), py = p.y(), pz = p.z();
    const double qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();

    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
      !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) ||
      !std::isfinite(qw))
    {
      fail("non-finite model pose in " + pose_topic_);
      return;
    }

    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!std::isfinite(norm) || norm < 1e-9) {
      fail("zero-norm quaternion in " + pose_topic_);
      return;
    }
    const double inv = 1.0 / norm;

    // Timestamp policy: strictly increasing Gazebo simulation time.
    const bool backwards =
      (stamp.sec() < last_sec_) ||
      (stamp.sec() == last_sec_ && stamp.nsec() < last_nsec_);
    const bool duplicate =
      (stamp.sec() == last_sec_ && stamp.nsec() == last_nsec_);
    if (have_last_stamp_) {
      if (backwards) {
        fail("simulation timestamp went backwards: " +
             std::to_string(last_sec_) + "." + std::to_string(last_nsec_) + " -> " +
             std::to_string(stamp.sec()) + "." + std::to_string(stamp.nsec()));
        return;
      }
      if (duplicate) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "duplicate Gazebo pose timestamp; skipping");
        return;
      }
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp.sec = static_cast<int32_t>(stamp.sec());
    out.header.stamp.nanosec = static_cast<uint32_t>(stamp.nsec());
    out.header.frame_id = "world";
    out.pose.position.x = px;
    out.pose.position.y = py;
    out.pose.position.z = pz;
    out.pose.orientation.x = qx * inv;
    out.pose.orientation.y = qy * inv;
    out.pose.orientation.z = qz * inv;
    out.pose.orientation.w = qw * inv;

    pub_->publish(out);
    last_sec_ = stamp.sec();
    last_nsec_ = stamp.nsec();
    have_last_stamp_ = true;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  gz::transport::Node gz_node_;
  std::string model_name_;
  std::string pose_topic_;
  std::string output_topic_;
  int64_t last_sec_{0};
  int32_t last_nsec_{0};
  bool have_last_stamp_{false};
  bool have_seen_model_{false};
  bool failed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyPoseConverter>());
  rclcpp::shutdown();
  return 0;
}

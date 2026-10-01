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

/// Task 3E minimal left/right foot contact converter.
///
/// Subscribes to the two internal raw ``ros_gz_interfaces/msg/Contacts``
/// topics produced by ros_gz_bridge from the native Gazebo contact sensors on
/// ``l_ankle`` / ``r_ankle`` and publishes the two frozen public
/// ``std_msgs/msg/Bool`` topics ``/left_foot/contact`` and
/// ``/right_foot/contact``.
///
/// Mapping is derived strictly from the scoped collision names, never from
/// array order, sensor start order or string prefixes:
///
///   ``true``  <=> a contact pair (foot main collision <-> ``ground_plane``)
///                is present and fresh in the latest simulation-time sample;
///   ``false`` <=> the matching sample is absent beyond the documented
///                simulation-time freshness window;
///   unknown   <=> no valid raw sample has been observed yet (nothing is
///                published, so ``false`` is never assumed before the sensor
///                has been proven to work).
///
/// The Gazebo contact sensor is contact-triggered: it is silent while a link
/// touches nothing (verified against gz-sim 8.15 ``ContactSensor::Publish``,
/// which only publishes when ``contact_size() > 0``). The frozen Bool is
/// therefore derived from *freshness of the matching simulation-time sample*:
/// the converter latches ``true`` while matching samples keep arriving and
/// releases to ``false`` only after ``stale_after`` seconds of **simulation**
/// time without a match. Freshness and republishing are driven exclusively by
/// the Gazebo ``/clock`` topic (simulation time); no wall timer and no wall
/// clock are used, so pausing freezes the state and never fabricates samples.
///
/// ``std_msgs/msg/Bool`` has no header; no timestamp is attached to it. Only
/// the raw ``Contacts`` message carries the Gazebo stamp, which is checked for
/// zero/backwards/duplicate and used for the freshness window.
///
/// This node never publishes /tf, never uses a controller and never writes to
/// the model. FSR / pressure / bumper / contact wrench interfaces are not
/// implemented.

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{
// Frozen public topics (Task 3A contract) - hardcoded to prevent drift.
constexpr char kLeftOutputTopic[] = "/left_foot/contact";
constexpr char kRightOutputTopic[] = "/right_foot/contact";

// Frozen semantic mapping (Task 3A contract).
constexpr char kModelName[] = "nao";
constexpr char kGroundModel[] = "ground_plane";
constexpr char kLeftLink[] = "l_ankle";
constexpr char kRightLink[] = "r_ankle";
constexpr char kLeftCollision[] = "l_ankle_collision";
constexpr char kRightCollision[] = "r_ankle_collision";

// Split a scoped gz collision name ("model::link::collision") on "::".
std::vector<std::string> split_scoped(const std::string & name)
{
  std::vector<std::string> parts;
  std::string cur;
  for (size_t i = 0; i < name.size(); ++i) {
    if (name[i] == ':' && i + 1 < name.size() && name[i + 1] == ':') {
      parts.push_back(cur);
      cur.clear();
      ++i;
    } else {
      cur.push_back(name[i]);
    }
  }
  parts.push_back(cur);
  return parts;
}
}  // namespace

class ContactConverter : public rclcpp::Node
{
public:
  ContactConverter()
  : rclcpp::Node("contact_converter")
  {
    left_raw_topic_ = this->declare_parameter<std::string>(
      "left_raw_topic", "/nao/left_foot_contact_raw");
    right_raw_topic_ = this->declare_parameter<std::string>(
      "right_raw_topic", "/nao/right_foot_contact_raw");
    update_rate_ = this->declare_parameter<double>("update_rate", 100.0);
    stale_after_ = this->declare_parameter<double>("stale_after", 0.05);
    if (update_rate_ <= 0.0) {
      update_rate_ = 100.0;
    }
    if (stale_after_ <= 0.0) {
      stale_after_ = 0.05;
    }

    rclcpp::QoS out_qos(rclcpp::KeepLast(1));
    out_qos.reliable().durability_volatile();
    rclcpp::QoS in_qos(rclcpp::KeepLast(1));
    in_qos.reliable().durability_volatile();

    pub_left_ = this->create_publisher<std_msgs::msg::Bool>(kLeftOutputTopic, out_qos);
    pub_right_ = this->create_publisher<std_msgs::msg::Bool>(kRightOutputTopic, out_qos);

    sub_left_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      left_raw_topic_, in_qos,
      [this](ros_gz_interfaces::msg::Contacts::SharedPtr msg) {on_raw(0, msg);});
    sub_right_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      right_raw_topic_, in_qos,
      [this](ros_gz_interfaces::msg::Contacts::SharedPtr msg) {on_raw(1, msg);});
    sub_clock_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
        on_clock(msg->clock.sec + msg->clock.nanosec * 1e-9);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "contact_converter: left '%s' -> '%s', right '%s' -> '%s' "
      "(model='%s', ground='%s', reliable, volatile, keep_last 1, "
      "update_rate=%.1f Hz, stale_after=%.3f s sim)",
      left_raw_topic_.c_str(), kLeftOutputTopic, right_raw_topic_.c_str(),
      kRightOutputTopic, kModelName, kGroundModel, update_rate_, stale_after_);
  }

private:
  struct FootState
  {
    bool seen{false};
    bool state{false};
    bool have_last_stamp{false};
    int64_t last_sec{0};
    uint32_t last_nsec{0};
    double last_match_sim{-1.0};
  };

  void fail(const std::string & reason)
  {
    if (!failed_) {
      failed_ = true;
      RCLCPP_ERROR(this->get_logger(), "CONTRACT VIOLATION: %s; publishing stopped",
                   reason.c_str());
    }
  }

  bool is_ground(const std::string & name) const
  {
    const std::vector<std::string> parts = split_scoped(name);
    return parts.size() >= 3 && parts[parts.size() - 3] == kGroundModel;
  }

  bool is_foot(const std::string & name, const char * link, const char * collision) const
  {
    const std::vector<std::string> parts = split_scoped(name);
    return parts.size() >= 3 && parts[parts.size() - 3] == kModelName &&
           parts[parts.size() - 2] == link && parts[parts.size() - 1] == collision;
  }

  void on_raw(size_t foot, const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    if (failed_) {
      return;
    }
    FootState & st = feet_[foot];
    const int64_t sec = msg->header.stamp.sec;
    const uint32_t nsec = msg->header.stamp.nanosec;
    if (sec == 0 && nsec == 0u) {
      return;  // no valid simulation time yet
    }
    if (st.have_last_stamp) {
      const bool backwards = (sec < st.last_sec) ||
        (sec == st.last_sec && nsec < st.last_nsec);
      const bool duplicate = (sec == st.last_sec && nsec == st.last_nsec);
      if (backwards) {
        fail(std::string("simulation timestamp went backwards on ") +
          (foot == 0 ? left_raw_topic_ : right_raw_topic_));
        return;
      }
      if (duplicate) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "duplicate contact timestamp on %s; skipping",
                             foot == 0 ? left_raw_topic_.c_str() : right_raw_topic_.c_str());
        return;
      }
    }
    st.last_sec = sec;
    st.last_nsec = nsec;
    st.have_last_stamp = true;
    st.seen = true;

    const char * link = (foot == 0) ? kLeftLink : kRightLink;
    const char * collision = (foot == 0) ? kLeftCollision : kRightCollision;
    bool matched = false;
    for (const auto & contact : msg->contacts) {
      const std::string & n1 = contact.collision1.name;
      const std::string & n2 = contact.collision2.name;
      const bool pair = (is_foot(n1, link, collision) && is_ground(n2)) ||
        (is_foot(n2, link, collision) && is_ground(n1));
      if (pair) {
        matched = true;
      } else if (n1.empty() || n2.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "contact on %s with missing collision name; ignored",
                             foot == 0 ? left_raw_topic_.c_str() : right_raw_topic_.c_str());
      }
    }
    if (matched) {
      st.last_match_sim = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    }
    // A non-matching (but valid) sample does NOT immediately force false and
    // never forces true; the simulation-time freshness window decides.
  }

  void on_clock(double sim_now)
  {
    if (failed_) {
      return;
    }
    sim_now_ = sim_now;
    if (sim_now < 0.0) {
      return;
    }
    if (last_pub_sim_ >= 0.0 && (sim_now - last_pub_sim_) < 1.0 / update_rate_) {
      return;
    }
    bool any = false;
    for (size_t i = 0; i < feet_.size(); ++i) {
      FootState & st = feet_[i];
      if (!st.seen) {
        continue;
      }
      const bool fresh = st.last_match_sim >= 0.0 &&
        (sim_now - st.last_match_sim) <= stale_after_;
      const bool new_state = fresh;
      const bool changed = (new_state != st.state);
      st.state = new_state;
      any = true;
      if (changed || last_pub_sim_ < 0.0) {
        RCLCPP_INFO(this->get_logger(), "%s foot contact = %s",
          (i == 0 ? "left" : "right"), new_state ? "true" : "false");
      }
      std_msgs::msg::Bool out;
      out.data = new_state;
      if (i == 0) {
        pub_left_->publish(out);
      } else {
        pub_right_->publish(out);
      }
    }
    if (any) {
      last_pub_sim_ = sim_now;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_left_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_right_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr sub_left_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr sub_right_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_clock_;
  std::array<FootState, 2> feet_{};
  std::string left_raw_topic_;
  std::string right_raw_topic_;
  double update_rate_{100.0};
  double stale_after_{0.05};
  double sim_now_{-1.0};
  double last_pub_sim_{-1.0};
  bool failed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContactConverter>());
  rclcpp::shutdown();
  return 0;
}

// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(const rclcpp::QoS & qos_profile)
  : Node("minimal_publisher"), qos_profile_(qos_profile)
  {


    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos_profile_, publisher_options_);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  // member variables
  rclcpp::QoS qos_profile_;
  rclcpp::PublisherOptions publisher_options_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

static const rmw_qos_profile_t my_custom_qos_profile =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,             // history
    10,                                            // depth
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,          // reliability
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,    // durability
    RMW_QOS_DEADLINE_DEFAULT,                     // deadline
    RMW_QOS_LIFESPAN_DEFAULT,                     // lifespan
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,     // liveliness
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,    // liveliness_lease_duration
    false                                         // avoid_ros_namespace_conventions
};

int main(int argc, char * argv[])
{
  //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  rclcpp::QoS qos_profile(
    rclcpp::QoSInitialization::from_rmw(my_custom_qos_profile), 
    my_custom_qos_profile
  );

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(qos_profile));
  rclcpp::shutdown();
  return 0;
}

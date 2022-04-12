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
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(const rclcpp::QoS & qos_profile)
  : Node("minimal_subscriber"), qos_profile_(qos_profile)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", qos_profile_, [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
      }, subscription_options_);      
  }

private:
  // member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions subscription_options_;
};

static const rmw_qos_profile_t my_custom_qos_profile =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};


int main(int argc, char * argv[])
{
 rclcpp::QoS qos_profile(
    rclcpp::QoSInitialization::from_rmw(my_custom_qos_profile), 
    my_custom_qos_profile
  );

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(qos_profile));
  rclcpp::shutdown();
  return 0;
}

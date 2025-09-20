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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

float previous_angular_z = 0;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      // TODO: use other topics, maybe even make this configurable?
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist & msg) const
  {
    // TODO: now only log output is given with this direction
    // Perhaps sending the direction on a new topic?
    if (previous_angular_z != (float)msg.angular.z)
    {
      if (msg.angular.z == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Going Straight : %f %f", previous_angular_z, msg.angular.z);
      }
      else if (msg.angular.z > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Going Left : %f %f", previous_angular_z, msg.angular.z);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Going Right : %f %f", previous_angular_z, msg.angular.z);
      }

      previous_angular_z = (float)msg.angular.z;
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

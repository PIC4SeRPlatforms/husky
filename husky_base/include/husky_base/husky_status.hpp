// Copyright 2023 Andrea Ostuni - PIC4SeR
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

#ifndef HUSKY_BASE__HUSKY_STATUS_HPP
#define HUSKY_BASE__HUSKY_STATUS_HPP

#include "husky_msgs/msg/husky_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace husky_status
{

class HuskyStatus : public rclcpp::Node
{
public:
  explicit HuskyStatus();

  void publish_status(husky_msgs::msg::HuskyStatus status_msg);

private:
  rclcpp::Publisher<husky_msgs::msg::HuskyStatus>::SharedPtr pub_status_;
};

}  // namespace husky_status

#endif  // HUSKY_BASE__HUSKY_STATUS_HPP

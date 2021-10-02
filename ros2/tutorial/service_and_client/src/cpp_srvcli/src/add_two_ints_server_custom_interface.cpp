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

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,
        std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>      response)
{
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld c: %ld" ,
            request->a, request->b, request->c);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");

    rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =
        node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}

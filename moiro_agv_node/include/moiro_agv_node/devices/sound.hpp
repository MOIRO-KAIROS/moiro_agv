// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Author: Darby Lim

#ifndef MOIRO_AGV_NODE__DEVICES__SOUND_HPP_
#define MOIRO_AGV_NODE__DEVICES__SOUND_HPP_

#include <moiro_interfaces/srv/sound.hpp>
#include <memory>
#include <string>
#include "moiro_agv_node/devices/devices.hpp"

namespace robotis
{
namespace moiro_agv
{
namespace devices
{
class Sound : public Devices
{
public:
  static void request(
    rclcpp::Client<moiro_interfaces::srv::Sound>::SharedPtr client,
    moiro_interfaces::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

private:
  rclcpp::Service<moiro_interfaces::srv::Sound>::SharedPtr srv_;
};
}  // namespace devices
}  // namespace moiro_agv
}  // namespace robotis
#endif  // moiro_agv_NODE__DEVICES__SOUND_HPP_

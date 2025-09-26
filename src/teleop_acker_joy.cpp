/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include "teleop_acker_joy/teleop_acker_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_acker_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopAckerJoy
 * directly into base nodes.
 */
struct TeleopAckerJoy::Impl
{

  // TODO: Perhaps individual message?
  static constexpr std::array lighting_command_names = {
    "headlight",
    "turn_left",
    "turn_right",
    "flash",
  };

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);
  void sendButtons(const sensor_msgs::msg::Joy::SharedPtr joy);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_vel_pub;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> cmd_lights_pub;

  bool require_enable_button;
  int64_t enable_button;
  int64_t enable_turbo_button;
  std::chrono::milliseconds failsafe_delay_ms;

  std::map<std::string, int64_t> button_map;
  std::map<std::string, int64_t> axis_map;
  std::map<std::string, std::map<std::string, double>> scale_map;
  std::map<std::string, double> offset_map;

  sensor_msgs::msg::Joy lastMessage{};
  std::map<std::string, bool> lightingStates{}; // could be an array, but we have the calc
  bool sent_disable_msg;
  rclcpp::Time last_non_zero_cmd{0};
  rclcpp::Clock clock;
};

/**
 * Constructs TeleopAckerJoy.
 */
TeleopAckerJoy::TeleopAckerJoy(const rclcpp::NodeOptions& options) : Node("teleop_acker_joy_node", options)
{
  pimpl_ = new Impl;

  const std::string topic_prefix = "cmd/"; // TODO: make parameter?

  // publisher
  pimpl_->cmd_vel_pub =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      topic_prefix + "vel",
      rclcpp::QoS(rclcpp::KeepLast(1)));

  std::map<std::string, int64_t> default_button_map;
  for (const auto& lighting_name : Impl::lighting_command_names)
  {
    const std::string publish_path = topic_prefix + lighting_name;
    // ROS_INFO_NAMED("TeleopAckerJoy", "publishing to %s", publish_path.c_str());
    pimpl_->cmd_lights_pub.emplace(lighting_name,
        this->create_publisher<std_msgs::msg::Bool>(
          publish_path,
          rclcpp::QoS(rclcpp::KeepLast(1))));

    default_button_map.emplace(lighting_name, -1L);
    pimpl_->lightingStates.emplace(lighting_name, false);
  }

  // subscriber
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&TeleopAckerJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  // config parameters
  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);
  pimpl_->enable_button = this->declare_parameter("enable_button", 5);
  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);
  pimpl_->failsafe_delay_ms = std::chrono::milliseconds{this->declare_parameter("failsafe_delay_ms", 0)};

  this->declare_parameters("lights", default_button_map);
  this->get_parameters("lights", pimpl_->button_map);

  std::map<std::string, int64_t> default_axis_map{
    {"linear", 5L},
    {"steering_angle", 6L},
    {"steering_angle_fine", -1L},
    {"steering_angle_velocity", -1L}
  };
  this->declare_parameters("axis", default_axis_map);
  this->get_parameters("axis", pimpl_->axis_map);

  std::map<std::string, double> default_scale_normal_map{
    {"linear", 0.5},
    {"steering_angle", 1.4},
    {"steering_angle_fine", 0.0},
    {"steering_angle_velocity", 0.0}
  };
  this->declare_parameters("scale", default_scale_normal_map);
  this->get_parameters("scale", pimpl_->scale_map["normal"]);

  std::map<std::string, double> default_scale_turbo_map{
    {"linear", 1.0},
    {"steering_angle", 0.8},
    {"steering_angle_fine", 0.0},
    {"steering_angle_velocity", 0.0}
  };
  this->declare_parameters("scale_turbo", default_scale_turbo_map);
  this->get_parameters("scale_turbo", pimpl_->scale_map["turbo"]);

  std::map<std::string, double> default_offset_map {
    {"linear", 0.0},
    {"steering_angle", 0.0},
    {"steering_angle_fine", 0.0},
    {"steering_angle_velocity", 0.4}
  };
  this->declare_parameters("offset", default_offset_map);
  this->get_parameters("offset", pimpl_->offset_map);

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "TeleopAckerJoy",
      "Teleop enable button %" PRId64 ".", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopAckerJoy",
    "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);

  for (const auto& [name, value] : pimpl_->button_map)
  {
    ROS_INFO_COND_NAMED(true || value != -1L, "TeleopAckerJoy",
      "button '%s' on %" PRId64,
      name.c_str(), value);
  }

  for (const auto& [name, value] : pimpl_->axis_map)
  {
    ROS_INFO_COND_NAMED(value != -1L, "TeleopAckerJoy",
      "axis '%s' on %" PRId64 " at scale %f with offs + %f.",
      name.c_str(), value, pimpl_->scale_map["normal"][name], pimpl_->offset_map[name]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && value != -1, "TeleopAckerJoy",
      "Turbo for axis '%s' is scale %f.",
      name.c_str(), pimpl_->scale_map["turbo"][name]);
  }


  pimpl_->sent_disable_msg = false;

  // callback if re-setting during runtime
  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {
      "lights.headlight",
      "lights.turnsignal_left",
      "lights.turnsignal_right",
      "lights.flash",
      "axis.linear",
      "axis.steering_angle",
      "axis.steering_angle_fine",
      "axis.steering_angle_velocity",
      "enable_button",
      "turbo_button",
      "failsafe_delay_ms",
    };
    static std::set<std::string> doubleparams = {
      "scale.linear", "scale_turbo.linear", "offset.linear",
      "scale.steering_angle", "scale_turbo.steering_angle", "offset.steering_angle",
      "scale.steering_angle_fine", "scale_turbo.steering_angle_fine", "offset.steering_angle_fine",
      "scale.steering_angle_velocity", "scale_turbo.steering_angle_velocity", "offset.steering_angle_velocity",
    };
    static std::set<std::string> boolparams = {
      "require_enable_button",
    };
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          result.successful = false;
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          result.successful = false;
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          return result;
        }
      }
      else if (boolparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
          result.successful = false;
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      const auto name = parameter.get_name();
      RCLCPP_INFO(this->get_logger(), "parsing parameter '%s'", name.c_str()
      );

      if (name == "require_enable_button")
      {
        this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
      }
      else if (name == "failsafe_delay_ms")
      {
        this->pimpl_->failsafe_delay_ms = std::chrono::milliseconds{parameter.get_value<rclcpp::PARAMETER_INTEGER>()};
      }
      else if (name == "enable_button")
      {
        this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (name == "enable_turbo_button")
      {
        this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (name.rfind("axis.", 0) != std::string::npos)
      {
        const auto which = name.substr(std::string("axis.").length());
        this->pimpl_->axis_map[which] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (name.rfind("lights.", 0) != std::string::npos)
      {
        const auto which = name.substr(std::string("lights.").length());
        this->pimpl_->button_map[which] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (name.rfind("scale.", 0) != std::string::npos)
      {
        auto which = name.substr(std::string("scale.").length());
        this->pimpl_->scale_map["normal"][which] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (name.rfind("scale_turbo.", 0) != std::string::npos)
      {
        auto which = name.substr(std::string("scale_turbo.").length());
        this->pimpl_->scale_map["turbo"][which] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (name.rfind("offset.", 0) != std::string::npos)
      {
        const auto which = name.substr(std::string("offset.").length());
        this->pimpl_->offset_map[which] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not required and thus is ignored", name.c_str());
      }
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopAckerJoy::~TeleopAckerJoy()
{
  delete pimpl_;
}

double getValScaled(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
              const std::map<std::string, int64_t>& axis_map,
              const std::map<std::string, double>& scale_map,
              const std::string& fieldname)
{
   if (axis_map.find(fieldname) == axis_map.end() ||
       axis_map.at(fieldname) == -1L ||
       scale_map.find(fieldname) == scale_map.end() ||
       static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
   {
     return 0.0;
   }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

bool getButton(const sensor_msgs::msg::Joy& joy_msg,
               const std::map<std::string, int64_t>& button_map,
               const std::string& fieldname)
{
  if (button_map.find(fieldname) == button_map.end() ||
      button_map.at(fieldname) == -1L ||
      static_cast<int>(joy_msg.buttons.size()) <= button_map.at(fieldname))
  {
    return false;
  }

  return joy_msg.buttons[button_map.at(fieldname)] > 0;
}

void TeleopAckerJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();

  cmd_vel_msg->drive.speed = offset_map["linear"] +
    getValScaled(joy_msg, axis_map, scale_map[which_map], "linear");
  cmd_vel_msg->drive.steering_angle = offset_map["steering_angle"] +
    getValScaled(joy_msg, axis_map, scale_map[which_map], "steering_angle") +
    getValScaled(joy_msg, axis_map, scale_map[which_map], "steering_angle_fine");
  cmd_vel_msg->drive.steering_angle_velocity = offset_map["steering_angle_velocity"] +
    std::abs(getValScaled(joy_msg, axis_map, scale_map[which_map], "steering_angle_velocity"));

  cmd_vel_pub->publish(std::move(cmd_vel_msg));
  sent_disable_msg = false;
}

void TeleopAckerJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (enable_turbo_button >= 0 &&
      static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
    last_non_zero_cmd = clock.now();
  }
  else if (!require_enable_button ||
           (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
           joy_msg->buttons[enable_button]))
  {
    sendCmdVelMsg(joy_msg, "normal");
    last_non_zero_cmd = clock.now();
  }
  else
  {
    // When enable button is released, send in some time a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      const bool is_zero_command_due = clock.now() > (last_non_zero_cmd + failsafe_delay_ms);
      if (is_zero_command_due)
      {
        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
        cmd_vel_pub->publish(std::move(cmd_vel_msg));
        sent_disable_msg = true;
        // TODO: Parameter "send hazard on disable-msg"
      }
    }
  }

  for (const auto& [button_name, _] : button_map)
  {
    const bool oldstate = getButton(lastMessage, button_map, button_name);
    const bool newstate = getButton(*joy_msg, button_map, button_name);
    // only on high transition
    if (!oldstate && newstate)
    {
      lightingStates.at(button_name) = !lightingStates.at(button_name);
      auto light_update = std::make_unique<std_msgs::msg::Bool>();
      light_update->data = lightingStates.at(button_name);
      cmd_lights_pub.at(button_name)->publish(std::move(light_update));
    }
  }

  lastMessage = *joy_msg;
}

}  // namespace teleop_acker_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_acker_joy::TeleopAckerJoy)

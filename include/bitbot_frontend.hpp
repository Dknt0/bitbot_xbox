/**
 * @file bitbot_frontend.hpp
 * @brief
 * @author Dknt
 * @date 2025.3
 */

#ifndef BITBOT_FRONTEND_HPP
#define BITBOT_FRONTEND_HPP

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocket.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>

#include "js_utils.hpp"
#include "xbox_js.hpp"

namespace xbox_js {

class JoystickFrontend {
 public:
  JoystickFrontend() = delete;
  ~JoystickFrontend() { is_connected_.store(false); }

  JoystickFrontend(const std::string& config_path) : controller_(config_path) {
    if (!std::filesystem::exists(config_path)) {
      std::cout << "Config file not found" << std::endl;
    }
    // Read config and relsase file
    {
      YAML::Node yaml_config = YAML::LoadFile(config_path);
      url_ = yaml_config["url"].as<std::string>();
      vel_scale_ = yaml_config["vel_scale"].as<std::vector<double>>();
    }
    is_connected_.store(false);

    // Setup websocket
    ix::initNetSystem();
    web_socket_.setUrl(url_);
    web_socket_.setOnMessageCallback(std::bind(
        &JoystickFrontend::MessageCallback, this, std::placeholders::_1));

    /// Setup joystick
    // Emergency shutdown
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.AxisValue(AxisName::LT) > 0.9)
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          web_socket_.send(ButtonMsg<"stop", "1">().value);
          std::cout << "Emergency Shutdown" << std::endl;
        },
        RumbleTemplate::emergency_shutdown);

    // Start connection
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (!is_connected_.load() && state.AxisValue(AxisName::DPadY) > 0.9)
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          is_connected_.store(true);
          web_socket_.start();
        },
        RumbleTemplate::success);

    // Start connection
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (is_connected_.load() && state.AxisValue(AxisName::DPadY) < -0.9)
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          is_connected_.store(false);
          web_socket_.stop();
        },
        RumbleTemplate::success);

    // Power on
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::Y))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          // Power on
          web_socket_.send(ButtonMsg<"power_on", "1">().value);
          web_socket_.send(ButtonMsg<"power_on", "2">().value);
          std::cout << "Power on" << std::endl;
          // Record data
          web_socket_.send(ButtonMsg<"enable_record", "1">().value);
          web_socket_.send(ButtonMsg<"enable_record", "2">().value);
        },
        RumbleTemplate::success);

    // Start state machine
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::B))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          web_socket_.send(ButtonMsg<"start", "1">().value);
          web_socket_.send(ButtonMsg<"start", "2">().value);
          std::cout << "Start state machine" << std::endl;
        },
        RumbleTemplate::success);

    // Init pose
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::A))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          web_socket_.send(ButtonMsg<"init_pose", "1">().value);
          web_socket_.send(ButtonMsg<"init_pose", "2">().value);
          std::cout << "Init pose" << std::endl;
        },
        RumbleTemplate::success);

    // Start inference
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.AxisValue(AxisName::RT) > 0.9)
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          web_socket_.send(ButtonMsg<"policy_run", "1">().value);
          web_socket_.send(ButtonMsg<"policy_run", "2">().value);
          std::cout << "Policy on" << std::endl;
        },
        RumbleTemplate::start_inference);

    // Velocity publish
    controller_.RegisterTiming(
        0.01, [this](const xbox_js::JoystickState& state) {
          if (is_connected_.load()) {
            web_socket_.send(VelocityMsg(
                "set_vel_x", vel_scale_[0] * state.AxisValue(AxisName::RY)));
            web_socket_.send(VelocityMsg(
                "set_vel_y", vel_scale_[1] * -state.AxisValue(AxisName::RX)));
            web_socket_.send(VelocityMsg(
                "set_vel_w", vel_scale_[2] * -state.AxisValue(AxisName::LX)));
          }
        });
  }

  /**
   * @brief Run the frontend. Blocking function
   *
   */
  void Run() {
    /*web_socket_.start();*/
    /*web_socket_.stop();*/
    controller_.Run();

    while (true) {
    }
  }

  void MessageCallback(const ix::WebSocketMessagePtr& msg) {
    if (msg->type == ix::WebSocketMessageType::Message) {
      std::cout << "received message: " << msg->str << std::endl;
      std::cout << "> " << std::flush;
    } else if (msg->type == ix::WebSocketMessageType::Open) {
      std::cout << "Connection established" << std::endl;
      std::cout << "> " << std::flush;
    } else if (msg->type == ix::WebSocketMessageType::Error) {
      // Maybe SSL is not configured properly
      std::cout << "Connection error: " << msg->errorInfo.reason << std::endl;
      std::cout << "> " << std::flush;
    }
  }

 private:
  std::string url_ = "ws://localhost:12888/console";
  std::atomic<bool> is_connected_;
  std::vector<double> vel_scale_;

  ix::WebSocket web_socket_;
  xbox_js::XBoxController controller_;
};

}  // namespace xbox_js

#endif  // !BITBOT_FRONTEND_HPP

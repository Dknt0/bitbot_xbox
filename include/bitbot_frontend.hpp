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
          constexpr std::string_view msg_1 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"stop\\\",\\\"value\\\":1}]}\"}";
          std::cout << "Emergency Shutdown" << std::endl;
          web_socket_.send(msg_1.data());
        },
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 70, .right = 70, .strong = 70, .weak = 70},
         .pulse = {.sustain_10ms = 15, .release_10ms = 0, .loop_count = 0}});

    // Start connectioe
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
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
         .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}});

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
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
         .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}});

    // Power on
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::Y))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          constexpr std::string_view msg_1 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"power_on\\\",\\\"value\\\":1}]}\"}";
          constexpr std::string_view msg_2 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"power_on\\\",\\\"value\\\":2}]}\"}";
          std::cout << "Power on" << std::endl;
          web_socket_.send(msg_1.data());
          web_socket_.send(msg_2.data());
          // Record data
          {
            constexpr std::string_view msg_1 =
                "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{"
                "\\\"name\\\":"
                "\\\"enable_record\\\",\\\"value\\\":1}]}\"}";
            constexpr std::string_view msg_2 =
                "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{"
                "\\\"name\\\":"
                "\\\"enable_record\\\",\\\"value\\\":2}]}\"}";
            std::cout << "Power on" << std::endl;
            web_socket_.send(msg_1.data());
            web_socket_.send(msg_2.data());
          }
        },
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
         .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}});

    // Start state machine
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::B))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          constexpr std::string_view msg_1 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"start\\\",\\\"value\\\":1}]}\"}";
          constexpr std::string_view msg_2 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"start\\\",\\\"value\\\":2}]}\"}";
          web_socket_.send(msg_1.data());
          web_socket_.send(msg_2.data());
          std::cout << "Start state machine" << std::endl;
        },
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
         .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}});

    // Init pose
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.ButtonValue(ButtonName::A))
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          constexpr std::string_view msg_1 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"init_pose\\\",\\\"value\\\":1}]}\"}";
          constexpr std::string_view msg_2 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"init_pose\\\",\\\"value\\\":2}]}\"}";
          web_socket_.send(msg_1.data());
          web_socket_.send(msg_2.data());
          std::cout << "Init pose" << std::endl;
        },
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
         .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}});

    // Start inference
    controller_.RegisterEvent(
        [this](const JoystickState& state) {
          if (this->is_connected_.load() && state.AxisValue(AxisName::RT) > 0.9)
            return true;
          else
            return false;
        },
        [this](const JoystickState& state) {
          constexpr std::string_view msg_1 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"policy_run\\\",\\\"value\\\":1}]}\"}";
          constexpr std::string_view msg_2 =
              "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":"
              "\\\"policy_run\\\",\\\"value\\\":2}]}\"}";
          web_socket_.send(msg_1.data());
          web_socket_.send(msg_2.data());
          std::cout << "Policy on" << std::endl;
        },
        {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
         .strength = {.left = 40, .right = 40, .strong = 40, .weak = 40},
         .pulse = {.sustain_10ms = 10, .release_10ms = 3, .loop_count = 2}});

    // Velocity publish
    controller_.RegisterTiming(0.01, [this](
                                         const xbox_js::JoystickState& state) {
      if (is_connected_.load()) {
        {
          double vel_x = vel_scale_[0] * state.AxisValue(AxisName::RY);
          int64_t vel_x_value = *reinterpret_cast<int64_t*>(&vel_x);
          /*auto res = *reinterpret_cast<double*>(&vel_x_value);*/
          /*std::cout << res << std::endl;*/
          std::string msg_x =
              "{\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"set_vel_x\\\","
              "\\\"value\\\":" +
              std::to_string(vel_x_value) + "}]}\",\"type\":\"events\"}";
          web_socket_.send(msg_x.c_str());
        }
        {
          double vel_y = vel_scale_[1] * -state.AxisValue(AxisName::RX);
          int64_t vel_y_value = *reinterpret_cast<int64_t*>(&vel_y);
          std::string msg_y =
              "{\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"set_vel_y\\\","
              "\\\"value\\\":" +
              std::to_string(vel_y_value) + "}]}\",\"type\":\"events\"}";
          web_socket_.send(msg_y.c_str());
        }
        {
          double vel_w = vel_scale_[2] * -state.AxisValue(AxisName::LX);
          int64_t vel_w_value = *reinterpret_cast<int64_t*>(&vel_w);
          std::string msg_w =
              "{\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"set_vel_w\\\","
              "\\\"value\\\":" +
              std::to_string(vel_w_value) + "}]}\",\"type\":\"events\"}";
          web_socket_.send(msg_w.c_str());
        }
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

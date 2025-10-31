/**
 * @file xbox_js.hpp
 * @brief Xbox joystick XBoxController
 * @author Dknt
 * @date 2025.3
 */

#ifndef XBOX_JS_HPP
#define XBOX_JS_HPP

#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define TERMINAL_PREFIX "\033[1;32m[Xbox JS]\033[0m "

namespace xbox_js {

enum AxisName {
  LX = 0,     // X Axis  left [-1, 1] right
  LY = 1,     // Y Axis  down [-1, 1] up
  LT = 2,     // Z Axis  free [0, 1] press
  RX = 3,     // Rx Axis  left [-1, 1] right
  RY = 4,     // Ry Axis  down [-1, 1] up
  RT = 5,     // Rz Axis  free [0, 1] press
  DPadX = 6,  // Hat0X  left {-1, 0, 1} right
  DPadY = 7   // Hat0Y  down {-1, 0, 1} up
};

enum ButtonName {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LB = 4,
  RB = 5,
  Disarm = 6,
  Arm = 7,
  Xbox = 8,
  LS = 9,
  RS = 10
};

class JoystickState {
  friend class XBoxController;

 public:
  double& Axis(size_t const idx) { return axes_buffer_[axis_map_[idx]]; }

  bool& Button(size_t const idx) { return buttons_buffer_[button_map_[idx]]; }

  double AxisValue(size_t const idx) const {
    return axes_buffer_[(*axis_map_.find(idx)).second];
  }

  bool ButtonValue(size_t const idx) const {
    return buttons_buffer_[(*button_map_.find(idx)).second];
  }

  void PrintState() const {
    auto current_time = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(current_time);
    std::cout << TERMINAL_PREFIX << "--- " << std::ctime(&time) << std::flush;
    std::cout << std::format(
                     "Axes: LX={:.2f} LY={:.2f} LT={:.2f} RX={:.2f} "
                     "RY={:.2f} RZ={:.2f} DPadX={:.2f} DPadY={:.2f}",
                     AxisValue(LX), AxisValue(LY), AxisValue(LT), AxisValue(RX),
                     AxisValue(RY), AxisValue(RT), AxisValue(DPadX),
                     AxisValue(DPadY))
              << std::endl;
    std::cout << std::format(
                     "Buttons: A={} B={} X={} Y={} LB={} RB={} Disarm={} "
                     "Arm={} Xbox={} LS={} RS={}",
                     ButtonValue(A), ButtonValue(B), ButtonValue(X),
                     ButtonValue(Y), ButtonValue(LB), ButtonValue(RB),
                     ButtonValue(Disarm), ButtonValue(Arm), ButtonValue(Xbox),
                     ButtonValue(LS), ButtonValue(RS))
              << std::endl;
  }

 private:
  double axes_buffer_[8] = {0};        // Axes
  bool buttons_buffer_[11] = {false};  // Buttons
  std::unordered_map<size_t, size_t> axis_map_;
  // TODO: Is their any bi-directional map in C++?
  std::unordered_map<size_t, size_t> reverse_axis_map_;
  std::unordered_map<size_t, size_t> button_map_;
  std::unordered_map<size_t, size_t> reverse_button_map_;

  void SetInputMap(const YAML::Node& config) {
    axis_map_.clear();
    button_map_.clear();
    // Axes
    axis_map_[AxisName::LX] = config["axis_map"]["LX"].as<int>();
    axis_map_[AxisName::LY] = config["axis_map"]["LY"].as<int>();
    axis_map_[AxisName::LT] = config["axis_map"]["LT"].as<int>();
    axis_map_[AxisName::RX] = config["axis_map"]["RX"].as<int>();
    axis_map_[AxisName::RY] = config["axis_map"]["RY"].as<int>();
    axis_map_[AxisName::RT] = config["axis_map"]["RT"].as<int>();
    axis_map_[AxisName::DPadX] = config["axis_map"]["DPadX"].as<int>();
    axis_map_[AxisName::DPadY] = config["axis_map"]["DPadY"].as<int>();

    // Buttons
    button_map_[ButtonName::A] = config["button_map"]["A"].as<int>();
    button_map_[ButtonName::B] = config["button_map"]["B"].as<int>();
    button_map_[ButtonName::X] = config["button_map"]["X"].as<int>();
    button_map_[ButtonName::Y] = config["button_map"]["Y"].as<int>();
    button_map_[ButtonName::LB] = config["button_map"]["LB"].as<int>();
    button_map_[ButtonName::RB] = config["button_map"]["RB"].as<int>();
    button_map_[ButtonName::Disarm] = config["button_map"]["Disarm"].as<int>();
    button_map_[ButtonName::Arm] = config["button_map"]["Arm"].as<int>();
    button_map_[ButtonName::Xbox] = config["button_map"]["Xbox"].as<int>();
    button_map_[ButtonName::LS] = config["button_map"]["LS"].as<int>();
    button_map_[ButtonName::RS] = config["button_map"]["RS"].as<int>();

    // Set reverse map
    for (const auto& [key, value] : axis_map_) reverse_axis_map_[value] = key;
    for (const auto& [key, value] : button_map_)
      reverse_button_map_[value] = key;
  }
};

struct JoystickRumblePack {
  char cmd = 0x03;  // Set to 0x03
  struct {
    u_char weak : 1;
    u_char strong : 1;
    u_char right : 1;
    u_char left : 1;
  } enable = {0, 0, 0, 0};  // 0 or 1
  struct {
    u_char left;
    u_char right;
    u_char strong;
    u_char weak;
  } strength = {0, 0, 0, 0};  // 0 ~ 100
  struct {
    u_char sustain_10ms;
    u_char release_10ms;
    u_char loop_count;  // 0 ~ 255
  } pulse = {0, 0, 0};
};

class XBoxController {
 public:
  using JsEvent = js_event;

  // Callback definition
  using EventCondition = std::function<bool(const JoystickState&)>;
  using EventCallback = std::function<void(const JoystickState&)>;
  using TimingCallback = std::function<void(const JoystickState&)>;

  struct Event {
    size_t id;
    bool last_state = false;
    EventCondition condition = nullptr;
    EventCallback callback = nullptr;
    JoystickRumblePack rumble_feedback;
  };

  struct Timing {
    size_t id;
    size_t time_cnt = 0;
    size_t period;
    TimingCallback callback = nullptr;
  };

 public:
  XBoxController() = delete;
  ~XBoxController() {
    running_.store(false);

    worker_thread_.join();
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  XBoxController(const std::string& config_path) {
    if (!std::filesystem::exists(config_path)) {
      std::cout << TERMINAL_PREFIX << "Config file not found" << std::endl;
    }

    YAML::Node yaml_config = YAML::LoadFile(config_path);

    fd_ = -1;
    dev_path_ = yaml_config["dev_path"].as<std::string>();
    hid_path_ = yaml_config["hid_path"].as<std::string>();
    debug_ = yaml_config["debug"].as<bool>();
    frequency_ = yaml_config["frequency"].as<uint>();
    enable_rumble_ = yaml_config["enable_rumble"].as<bool>();
    period_ = 1.0 / frequency_;

    joystick_state_.SetInputMap(yaml_config);

    // O_NONBLOCK open
    fd_ = open(dev_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      fd_ = -1;
      std::cout << TERMINAL_PREFIX
                << "Falied to open joytick device: " << dev_path_ << std::endl;
    }

    // Read joystick info
    // ioctl() is used to perform device-specific input/output operations
    unsigned char num_axis_tmp;
    unsigned char num_button_tmp;
    ioctl(fd_, JSIOCGVERSION, &version_);
    ioctl(fd_, JSIOCGAXES, &num_axis_tmp);
    ioctl(fd_, JSIOCGBUTTONS, &num_button_tmp);
    ioctl(fd_, JSIOCGNAME(512), name_);

    num_axis_ = num_axis_tmp;
    num_button_ = num_button_tmp;
    if (debug_) {
      std::cout << TERMINAL_PREFIX << "Driver version: " << (version_ >> 16)
                << "." << ((version_ >> 8)) << "." << (version_ & 0xff)
                << std::endl;
      std::cout << TERMINAL_PREFIX << "Joystick name: " << name_ << std::endl;
      std::cout << TERMINAL_PREFIX << "Number of axes: " << num_axis_
                << std::endl;
      std::cout << TERMINAL_PREFIX << "Number of buttons: " << num_button_
                << std::endl;
    }

    // Get HID handle
    if (enable_rumble_) {
      hidraw_ = open(hid_path_.c_str(), O_WRONLY);
      if (hidraw_ < 0) {
        std::cout << TERMINAL_PREFIX << "Error opening " << hid_path_
                  << ". Changing permission." << std::endl;
        system((std::string("sudo chmod 666 ") + hid_path_).c_str());
        hidraw_ = open(hid_path_.c_str(), O_WRONLY);

        if (hidraw_ < 0) {
          std::cout << TERMINAL_PREFIX << "Error hid path." << std::endl;
        }
      }
    }
  }

  /**
   * @brief Register event callback
   *
   * @param[in] condition Condiction function
   * @param[in] callback State callbac
   * @param[in] rumble_pack Feedback rumble pac
   * @return Event id
   */
  size_t RegisterEvent(EventCondition condition, EventCallback callback,
                       JoystickRumblePack const& rumble_pack = {}) {
    events_.push_back({.id = events_.size(),
                       .last_state = false,
                       .condition = condition,
                       .callback = callback,
                       .rumble_feedback = rumble_pack});
    return events_.back().id;
  }

  /**
   * @brief Register timing callback
   *
   * @param[in] period Period in seconds
   * @param[in] callback Callback function
   * @return Timing id
   */
  size_t RegisterTiming(double const period, TimingCallback callback) {
    timings_.push_back({.id = timings_.size(),
                        .time_cnt = 0,
                        .period = size_t(period / period_),
                        .callback = callback});
    return timings_.back().id;
  }

  /**
   * @brief Run the controller loop
   *
   */
  void Run() {
    running_.store(true);
    worker_thread_ = std::thread(&XBoxController::WorkerThread, this);
  }

  /**
   * @brief Stop the controller loop
   *
   */
  void Stop() { running_.store(false); }

 private:
  /**
   * @brief Update joystick state basd on JsEvent
   *
   * @param[in] js JsEvent from device
   */
  void UpdateState(const JsEvent& js) {
    switch (js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_BUTTON:
        if (joystick_state_.reverse_button_map_.find(js.number) !=
            joystick_state_.reverse_button_map_.end())
          joystick_state_.buttons_buffer_[js.number] = js.value;
        if (debug_)
          std::cout << TERMINAL_PREFIX << "Button: " << int(js.number)
                    << std::endl;
        break;
      case JS_EVENT_AXIS:
        if (joystick_state_.reverse_axis_map_.find(js.number) !=
            joystick_state_.reverse_axis_map_.end())
          switch (joystick_state_.reverse_axis_map_[js.number]) {
            case AxisName::LX:
              [[fallthrough]];
            case AxisName::RX:
              joystick_state_.axes_buffer_[js.number] =
                  double(js.value) / 32767.0;
              break;
            case AxisName::LY:
              [[fallthrough]];
            case AxisName::RY:
              joystick_state_.axes_buffer_[js.number] =
                  -double(js.value) / 32767.0;
              break;
            case AxisName::LT:
              [[fallthrough]];
            case AxisName::RT:
              joystick_state_.axes_buffer_[js.number] =
                  double(js.value + 32767) / 65534.0;
              break;
            case AxisName::DPadX:
              joystick_state_
                  .axes_buffer_[joystick_state_.axis_map_[js.number]] =
                  (0 < js.value) - (js.value < 0);
              break;
            case AxisName::DPadY:
              joystick_state_
                  .axes_buffer_[joystick_state_.axis_map_[js.number]] =
                  -(0 < js.value) + (js.value < 0);
              break;
          }
        if (debug_)
          std::cout << TERMINAL_PREFIX << "Axis: " << int(js.number)
                    << std::endl;
        break;
    }
  }

  /**
   * @brief Worker thread function. Read from device and check events
   *
   */
  void WorkerThread() {
    if (debug_) std::cout << "Worker thread started" << std::endl;
    auto next_wake = std::chrono::system_clock::now();

    while (running_.load()) {
      {
        // Lock for unsynchronized accessing
        std::lock_guard<std::mutex> lock(state_mutex_);
        Read();
        // Check events
        for (auto& event : events_) {
          if (event.condition(joystick_state_)) {
            if (event.last_state == false) {
              event.last_state = true;
              event.callback(joystick_state_);
              Rumble(event.rumble_feedback);
              break;
            }
          } else {
            event.last_state = false;
          }
        }
        // Check timings
        for (auto& timing : timings_) {
          if (timing.time_cnt % timing.period == 0) {
            timing.callback(joystick_state_);
            timing.time_cnt = 0;
          }
          timing.time_cnt++;
        }
      }
      /*std::this_thread::sleep_for(*/
      /*    std::chrono::microseconds(int(period_ * 1000000)));*/
      next_wake += std::chrono::microseconds(int(period_ * 1000000));
      std::this_thread::sleep_until(next_wake);
    }
    if (debug_) std::cout << "Worker thread stopped" << std::endl;
  }

  /**
   * @brief Read from device
   *
   */
  void Read() {
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;
    // Readiness state of files
    fd_set rfds;
    JsEvent js;
    int len = -1;

    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    int ret = select(fd_ + 1, &rfds, NULL, NULL, &timeout);
    if (ret > 0 && FD_ISSET(fd_, &rfds)) {
      // Reset the js_event
      memset(&js, 0, sizeof(js));
      // Read new js_event from file descriptor
      len = read(fd_, &js, sizeof(JsEvent));
      // Check size
      if (len != sizeof(JsEvent)) {
        if (debug_)
          std::cout << TERMINAL_PREFIX << "XBoxJoystick error readings"
                    << std::endl;
        return;
      }

      UpdateState(js);
      if (debug_) {
        joystick_state_.PrintState();
      }
    }
  }

  void Rumble(const JoystickRumblePack& rumble_pack) {
    std::cout << (enable_rumble_ ? "True" : "False") << std::endl;
    if (hidraw_ < 0 || !enable_rumble_) {
    } else {
      int ret = write(hidraw_, &rumble_pack, sizeof(JoystickRumblePack));
    }
  }

 private:
  std::string dev_path_ = "";  // Device path
  std::string hid_path_ = "";  // HID path
  int hidraw_ = 0;             // Human Interface Devices handle
  int fd_ = -1;                // File descriptor used by system call
  uint frequency_ = 1000;
  double period_ = 1.0 / frequency_;

  bool debug_ = false;
  int version_ = 0x000800;     // Joystick version
  char name_[512] = "Unkown";  // Joystick name
  size_t num_axis_ = 0;        // Number of axes
  size_t num_button_ = 0;      // Number of buttons

  std::mutex state_mutex_;        // Mutex for state
  JoystickState joystick_state_;  // State

  std::thread worker_thread_;        // Thread to read from device
  std::atomic_bool running_{false};  // Thread running flag
  bool enable_rumble_ = false;
  std::vector<Event> events_;    // Events list
  std::vector<Timing> timings_;  // Timings list
};
}  // namespace xbox_js

#endif  // !XPAD_JS_HPP

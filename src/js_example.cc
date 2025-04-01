#include "xbox_js.hpp"

int main() {
  std::string config_path = "/home/dknt/Project/xpad_dev/config/default.yaml";
  xbox_js::XBoxController controller(config_path);

  // X button
  controller.RegisterEvent(
      [](const xbox_js::JoystickState& state) {
        if (state.ButtonValue(xbox_js::X))
          return true;
        else
          return false;
      },
      [](const xbox_js::JoystickState& state) {
        std::cout << "X button pressed" << std::endl;
      },
      {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
       .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
       .pulse = {.sustain_10ms = 5, .release_10ms = 0, .loop_count = 0}});

  // LB&RB button
  controller.RegisterEvent(
      [](const xbox_js::JoystickState& state) {
        if (state.ButtonValue(xbox_js::LB) && state.ButtonValue(xbox_js::RB))
          return true;
        else
          return false;
      },
      [](const xbox_js::JoystickState& state) {
        std::cout << "LB and RB button pressed" << std::endl;
      },
      {.enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
       .strength = {.left = 70, .right = 70, .strong = 0, .weak = 0},
       .pulse = {.sustain_10ms = 10, .release_10ms = 20, .loop_count = 2}});

  // Timing
  controller.RegisterTiming(1.0, [](const xbox_js::JoystickState& state) {
    static size_t cnt = 0;
    std::cout << "Timing callback: " << cnt++ << " s." << std::endl;
    /*state.PrintState();*/
  });

  controller.Run();

  while (true) {
  }

  return 0;
}

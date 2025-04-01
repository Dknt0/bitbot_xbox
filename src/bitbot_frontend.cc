#include "bitbot_frontend.hpp"

int main() {
  xbox_js::JoystickFrontend frontend(
      "/home/dknt/Project/xpad_dev/config/frontend.yaml");

  frontend.Run();
  return 0;
}

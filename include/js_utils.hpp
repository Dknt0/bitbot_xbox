#ifndef JS_UTILS_HPP
#define JS_UTILS_HPP

#include "xbox_js.hpp"

namespace xbox_js {

namespace RumbleTemplate {
constexpr JoystickRumblePack success = {
    .enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
    .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
    .pulse = {.sustain_10ms = 10, .release_10ms = 0, .loop_count = 0}};

constexpr JoystickRumblePack failure = {
    .enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
    .strength = {.left = 50, .right = 50, .strong = 50, .weak = 50},
    .pulse = {.sustain_10ms = 5, .release_10ms = 0, .loop_count = 1}};

constexpr JoystickRumblePack start_inference = {
    .enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
    .strength = {.left = 40, .right = 40, .strong = 40, .weak = 40},
    .pulse = {.sustain_10ms = 10, .release_10ms = 3, .loop_count = 2}};

constexpr JoystickRumblePack emergency_shutdown = {
    .enable = {.weak = 1, .strong = 1, .right = 1, .left = 1},
    .strength = {.left = 70, .right = 70, .strong = 70, .weak = 70},
    .pulse = {.sustain_10ms = 10, .release_10ms = 5, .loop_count = 1}};
}  // namespace RumbleTemplate

template <size_t N>
struct CTString {
  constexpr CTString(const char (&str)[N]) { std::copy_n(str, N, value); }
  constexpr size_t size() const { return N; }
  char value[N]{};
};

template <CTString Str>
constexpr auto CTStringCat() {
  return Str;
}

template <CTString Str1, CTString Str2, CTString... Ctrs>
constexpr auto CTStringCat() {
  auto tail = CTStringCat<Str2, Ctrs...>();
  char front[Str1.size() + tail.size() - 1] = {};
  std::copy_n(Str1.value, Str1.size() - 1, front);
  std::copy_n(tail.value, tail.size(), front + Str1.size() - 1);
  CTString res(front);
  return res;
}

template <CTString Name, CTString Value>
constexpr auto ButtonMsg() {
  constexpr CTString str_1(
      "{\"type\":\"events\",\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"");
  constexpr CTString str_2("\\\",\\\"value\\\":");
  constexpr CTString str_3("}]}\"}");
  constexpr auto res = CTStringCat<str_1, Name, str_2, Value, str_3>();
  return res;
}

inline const std::string VelocityMsg(const std::string& name, double value) {
  int64_t vel_value = *reinterpret_cast<int64_t*>(&value);
  std::string msg = "{\"data\":\"{\\\"events\\\":[{\\\"name\\\":\\\"" + name +
                    "\\\",\\\"value\\\":" + std::to_string(vel_value) +
                    "}]}\",\"type\":\"events\"}";
  return msg;
}

}  // namespace xbox_js

#endif  // !JS_UTILS_HPP

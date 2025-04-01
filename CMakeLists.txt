cmake_minimum_required(VERSION 3.20.0)
project(xbox_joystick)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/include
)

add_library(xbox_js src/xbox_js.cc)
target_link_libraries(xbox_js
  pthread
  ${YAML_CPP_LIBRARIES}
)

set(BUILD_TEST ON)
if(BUILD_TEST)
  find_package(Curses REQUIRED)
  include_directories(
    ${CURSES_INCLUDE_DIR}
  )
  add_executable(js_example src/js_example.cc)
  target_link_libraries(js_example
    xbox_js
  )
  add_executable(rumble_test src/rumble_test.cc)
  target_link_libraries(rumble_test
    ${CURSES_LIBRARIES}
  )
endif()

set(BUILD_FRONTEND ON)
if(BUILD_FRONTEND)
  include(FetchContent)
  set(IXWebSocket_SOURCE_DIR ${CMAKE_SOURCE_DIR}/third_party/IXWebSocket)
  FetchContent_Declare(
    IXWebSocket 
    GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
    GIT_TAG v11.4.5
    SOURCE_DIR ${IXWebSocket_SOURCE_DIR}
  )
  FetchContent_MakeAvailable(IXWebSocket)

  add_executable(bitbot_frontend src/bitbot_frontend.cc)
  target_link_libraries(bitbot_frontend
    xbox_js
    ixwebsocket
  )
endif()


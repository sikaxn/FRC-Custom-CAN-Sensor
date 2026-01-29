cmake_minimum_required(VERSION 3.20)
project(ledsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.12.0
)
FetchContent_MakeAvailable(pybind11)

set(SIM_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include)
set(ARDUINO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../Arduino/BatteryLEDCombo)

pybind11_add_module(ledsim
  cpp/sim_bindings.cpp
  cpp/sim_time.cpp
  cpp/sim_fastled.cpp
  cpp/sim_globals.cpp
  ${ARDUINO_SRC}/mode.cpp
  ${ARDUINO_SRC}/pattern.cpp
)

target_include_directories(ledsim PRIVATE
  ${SIM_INCLUDE}
  ${ARDUINO_SRC}
)

target_compile_definitions(ledsim PRIVATE LEDSIM_HOST)

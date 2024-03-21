cmake_minimum_required(VERSION 3.8)
project(xsens_mtw_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fms-extensions)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)
link_directories(lib)

add_executable(mt_w_manager
  src/mt_w_main.cpp
  src/mastercallback.cpp
  src/mtwcallback.cpp
  src/findClosestUpdateRate.cpp
  src/conio.c
  src/console.cpp
  src/myxda.cpp
)

target_link_libraries(mt_w_manager
  xsensdeviceapi
  xstypes
  pthread
  dl
)

ament_target_dependencies(mt_w_manager
rclcpp
std_msgs
sensor_msgs
geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY lib/
  DESTINATION lib
)

install(
  TARGETS mt_w_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(blimp_interfaces REQUIRED)
#find_package(vector REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(dynamic_model src/inv_kine.cpp)
ament_target_dependencies(dynamic_model
  rclcpp
  blimp_interfaces
  Eigen3
)

add_executable(F_to_Esc src/force_to_ESC_input.cpp)
ament_target_dependencies(F_to_Esc
  rclcpp
  blimp_interfaces
  Eigen3
)

add_executable(pi_controller src/Balloon_pi.cpp)
ament_target_dependencies(pi_controller
  rclcpp
  blimp_interfaces
)

add_executable(old_cam src/old_cam.cpp)
ament_target_dependencies(old_cam rclcpp blimp_interfaces OpenCV sensor_msgs)

add_executable(ES_control src/Extremum_Seeking.cpp)
ament_target_dependencies(ES_control rclcpp blimp_interfaces Eigen3)

install(TARGETS
  dynamic_model
  F_to_Esc
  pi_controller
  old_cam
  ES_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

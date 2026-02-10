cmake_minimum_required(VERSION 3.8)
project(blimp_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EscInput.msg"
  "msg/CameraCoord.msg"
  "msg/ImuData.msg"
  "msg/BaroData.msg"
  "msg/LidarData.msg"
  "msg/CartCoord.msg"
  "msg/UtcTime.msg"
  "msg/Bool.msg"
  "srv/GetCamera.srv"
  "srv/Detection.srv"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()

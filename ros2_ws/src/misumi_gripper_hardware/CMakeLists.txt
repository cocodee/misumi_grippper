cmake_minimum_required(VERSION 3.8)
project(misumi_gripper_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)
 # Find the system-installed libmodbus

# Add the hardware interface library
add_library(misumi_gripper_hardware_plugin SHARED src/misumi_gripper_hardware.cpp)
target_include_directories(misumi_gripper_hardware_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../src/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(misumi_gripper_hardware_plugin
  hardware_interface
  pluginlib
  rclcpp
)
target_link_libraries(misumi_gripper_hardware_plugin misumi_gripper_lib.a)

pluginlib_export_plugin_description_file(hardware_interface src/misumi_gripper.xml)

install(TARGETS
  misumi_gripper_hardware_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  include/
  DESTINATION include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME})

install(
  FILES misumi_gripper.xml
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()

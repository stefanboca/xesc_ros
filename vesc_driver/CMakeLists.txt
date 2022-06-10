cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_library(
  vesc_driver SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
ament_target_dependencies(vesc_driver rclcpp rclcpp_components std_msgs vesc_msgs serial)

if(NOT WIN32)
  ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(vesc_driver_node src/vesc_driver_node.cpp)
target_link_libraries(vesc_driver_node vesc_driver)
ament_target_dependencies(vesc_driver_node rclcpp)

install(
  TARGETS
  vesc_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  vesc_driver_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# # node executable
# add_executable(vesc_driver_node src/vesc_driver_node.cpp)
# add_dependencies(vesc_driver_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vesc_driver_node
#   vesc_driver
#   ${catkin_LIBRARIES}
# )

# # nodelet library
# add_library(vesc_driver_nodelet src/vesc_driver_nodelet.cpp)
# add_dependencies(vesc_driver_nodelet ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vesc_driver_nodelet
#   vesc_driver
#   ${catkin_LIBRARIES}
# )

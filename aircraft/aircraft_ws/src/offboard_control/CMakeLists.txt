cmake_minimum_required(VERSION 3.8)
project(offboard_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autopilot_interface_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ground_system_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Add the CMake module path
if(NOT DEFINED GEOGRAPHICLIB_CMAKE_DIR)
  set(GEOGRAPHICLIB_CMAKE_DIR "/usr/share/cmake/geographiclib")
endif()
set(CMAKE_MODULE_PATH "${GEOGRAPHICLIB_CMAKE_DIR}" ${CMAKE_MODULE_PATH})
find_package(GeographicLib REQUIRED)

add_executable(px4_offboard src/px4_offboard.cpp)
target_link_libraries(px4_offboard ${GeographicLib_LIBRARIES})
target_include_directories(px4_offboard PRIVATE ${GeographicLib_INCLUDE_DIRS})
ament_target_dependencies(px4_offboard 
                          rclcpp
                          rclcpp_action
                          px4_msgs
                          geometry_msgs
                          autopilot_interface_msgs
                          std_msgs
                          nav_msgs
                          ground_system_msgs
                          vision_msgs
                          )

add_executable(ardupilot_guided src/ardupilot_guided.cpp)
target_link_libraries(ardupilot_guided ${GeographicLib_LIBRARIES})
target_include_directories(ardupilot_guided PRIVATE ${GeographicLib_INCLUDE_DIRS})
ament_target_dependencies(ardupilot_guided
                          rclcpp
                          rclcpp_action
                          mavros_msgs
                          geometry_msgs
                          nav_msgs
                          sensor_msgs
                          autopilot_interface_msgs
                          geographic_msgs
                          std_msgs
                          ground_system_msgs
                          vision_msgs
                          )

install(TARGETS
  px4_offboard
  ardupilot_guided
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(mavros_xyz_position_offboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

# The mission core is a plain C++17 library and is also buildable without ROS.
set(MISSION_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../mission_core
  ${CMAKE_CURRENT_BINARY_DIR}/mission_core EXCLUDE_FROM_ALL)

add_library(${PROJECT_NAME}_core
  src/common/artifact_log.cpp
  src/common/cli.cpp
  src/common/types.cpp
  src/bridge/lcp_vision_bridge.cpp
  src/application/application_node.cpp
  src/communication/ground_station_link.cpp
  src/initialization/initialization.cpp
  src/navigation/trajectory_planner.cpp
  src/offboard/offboard.cpp
  src/execution/px4_action_executor.cpp)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs mavros_msgs lslidar_msgs)
target_link_libraries(${PROJECT_NAME}_core PkgConfig::JSONCPP mission_core)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mavros_xyz_position_node src/main.cpp)
target_link_libraries(mavros_xyz_position_node ${PROJECT_NAME}_core)
ament_target_dependencies(mavros_xyz_position_node rclcpp)
target_compile_options(mavros_xyz_position_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}_core mavros_xyz_position_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_trajectory test/test_navigation_trajectory.cpp)
  if(TARGET test_navigation_trajectory)
    target_link_libraries(test_navigation_trajectory ${PROJECT_NAME}_core)
    target_compile_options(test_navigation_trajectory PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_udp_generic test/test_udp_generic.cpp)
  if(TARGET test_udp_generic)
    target_link_libraries(test_udp_generic ${PROJECT_NAME}_core)
    target_compile_options(test_udp_generic PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_core_components test/test_core_components.cpp SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET test_core_components)
    target_link_libraries(test_core_components ${PROJECT_NAME}_core)
    ament_target_dependencies(test_core_components rclcpp)
    target_compile_options(test_core_components PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_px4_action_executor test/test_px4_action_executor.cpp SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET test_px4_action_executor)
    target_link_libraries(test_px4_action_executor ${PROJECT_NAME}_core)
    ament_target_dependencies(test_px4_action_executor rclcpp)
    target_compile_options(test_px4_action_executor PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_package()

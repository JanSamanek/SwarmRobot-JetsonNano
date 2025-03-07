cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slg_msgs REQUIRED)
find_package(tracker_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

set(source
  src/controller_node.cpp
  src/pid_controller.cpp
  src/apf_controller.cpp
  src/controller_utils.cpp
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED ${source})
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name}
  PUBLIC
  rclcpp::rclcpp
  ${slg_msgs_TARGETS}
  slg_msgs::slg_core
  ${tracker_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nlohmann_json_TARGETS}
  tf2::tf2 
  tf2_geometry_msgs::tf2_geometry_msgs 
)

set(executable_name ${PROJECT_NAME}_node)

add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name}
  PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

target_compile_features(${executable_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

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

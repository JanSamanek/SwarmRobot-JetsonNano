cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slg_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(source
  src/main.cpp
  src/utils.cpp
  src/tracker_node.cpp
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
  ${sensor_msgs_TARGETS}
  ${slg_msgs_TARGETS}
  slg_msgs::slg_core
  ${visualization_msgs_TARGETS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

target_compile_features(tracker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS tracker
  DESTINATION lib/${PROJECT_NAME})

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

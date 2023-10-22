cmake_minimum_required(VERSION 3.8)
project(turpong)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(main src/main.cpp)
add_executable(keybr src/keybr.cpp)

ament_target_dependencies(main rclcpp std_msgs turtlesim geometry_msgs)
ament_target_dependencies(keybr rclcpp std_msgs)

install(TARGETS
  main
  keybr
  DESTINATION lib/${PROJECT_NAME})

ament_package()

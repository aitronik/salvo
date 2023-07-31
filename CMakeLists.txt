cmake_minimum_required(VERSION 3.5)
project(log_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# find_package(ncurses REQUIRED)


include_directories(
  include
)

add_executable(data_subscriber src/main.cpp)
ament_target_dependencies(data_subscriber rclcpp std_msgs rcl_interfaces)
target_link_libraries(data_subscriber ncurses)
 

install(TARGETS
  data_subscriber
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
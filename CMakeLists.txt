cmake_minimum_required(VERSION 3.11)
project(salvo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

set(ROS2_SUBSCRIBE 1)


if(DEFINED ROS2_SUBSCRIBE)
  message("ROS2_SUBSCRIBE defined")
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rcl_interfaces REQUIRED)

  add_library(salvo_lib INTERFACE)
  target_include_directories(salvo_lib INTERFACE 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>"
  )
  ament_export_targets(export_salvo_lib HAS_LIBRARY_TARGET)

  ## ###################### ROS2 Library %% SALVO ######################  

  add_library(ros2_subscriber src/impl/ros2_subscriber.cpp)
  ament_target_dependencies(ros2_subscriber rclcpp std_msgs rcl_interfaces)
  add_executable(data_publisher src/run_data_publisher.cpp)
  ament_target_dependencies(data_publisher rclcpp std_msgs rcl_interfaces)
  add_executable(rostest src/run_test.cpp)
  ament_target_dependencies(rostest rclcpp std_msgs rcl_interfaces)

  add_executable(salvo src/run_salvo.cpp)
  target_link_libraries(salvo ros2_subscriber)

  install(TARGETS data_publisher rostest salvo
          DESTINATION lib/${PROJECT_NAME})

  install(TARGETS salvo_lib
          EXPORT export_salvo_lib
          DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY include/salvo/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".git*" EXCLUDE
  )

  ## ###################### QT ######################
  
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
  QT5_WRAP_CPP(MOC_SOURCES include/impl/qt_gui.h)
  
  add_executable(guardo src/run_guardo.cpp src/impl/qt_gui.cpp ${MOC_SOURCES} )
  target_link_libraries(guardo ros2_subscriber Qt5::Core Qt5::Widgets pthread)
 
  install(
    TARGETS guardo
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
endif()
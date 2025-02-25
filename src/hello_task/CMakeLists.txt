cmake_minimum_required(VERSION 3.8)
project(hello_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(generate_parameter_library REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(customized_interfaces REQUIRED)

generate_parameter_library(
  sine_wave_param 
  config/sine_wave_param.yaml 
)

add_executable(sine_wave_publisher src/hello_task_publisher.cpp)
target_link_libraries(sine_wave_publisher
    sine_wave_param
    rclcpp::rclcpp
)
ament_target_dependencies(sine_wave_publisher rclcpp std_msgs)

add_executable(sine_wave_subscriber src/hello_task_subscriber.cpp)
ament_target_dependencies(sine_wave_subscriber rclcpp std_msgs)

add_executable(grayscale_client src/grayscale_client.cpp)
ament_target_dependencies(grayscale_client rclcpp std_msgs sensor_msgs cv_bridge OpenCV customized_interfaces)

add_executable(grayscale_server src/grayscale_server.cpp)
ament_target_dependencies(grayscale_server rclcpp std_msgs sensor_msgs cv_bridge OpenCV customized_interfaces)

install(TARGETS
  sine_wave_publisher
  sine_wave_subscriber
  grayscale_server
  grayscale_client
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
  )  

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_basic_test test/basic_test.cpp)

  target_include_directories(${PROJECT_NAME}_basic_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_basic_test
    sine_wave_param
    rclcpp::rclcpp
)
  ament_target_dependencies(${PROJECT_NAME}_basic_test rclcpp std_msgs)
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

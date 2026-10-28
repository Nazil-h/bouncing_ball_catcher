cmake_minimum_required(VERSION 3.20)
project(bouncing_ball_catcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(COMMON_INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(COMMON_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(camera_node src/camera_node.cpp)
target_compile_features(camera_node PUBLIC cxx_std_20)
target_include_directories(camera_node PRIVATE ${COMMON_INCLUDE_DIRS})
target_link_libraries(camera_node ${COMMON_LIBRARIES})

add_executable(game_node src/game_node.cpp)
target_compile_features(game_node PUBLIC cxx_std_20)
target_include_directories(game_node PRIVATE ${COMMON_INCLUDE_DIRS})
target_link_libraries(game_node ${COMMON_LIBRARIES})

install(TARGETS camera_node game_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(ros2_gazebo_tut)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(turtlebot3_gazebo REQUIRED)

# Adding an executable for code to avoid obstacles
add_executable(walker src/walker.cpp src/main.cpp)
ament_target_dependencies(walker rclcpp sensor_msgs geometry_msgs)
target_include_directories(walker PRIVATE include)
target_link_libraries(walker ${ament_LIBRARIES})

install(TARGETS
  walker
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()

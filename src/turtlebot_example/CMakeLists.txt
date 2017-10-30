cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(turtlebot_example_node src/turtlebot_example_node.cpp)
add_executable(state_estimation_node src/state_estimation.cpp)

target_link_libraries(turtlebot_example_node
  ${catkin_LIBRARIES}
)

target_link_libraries(state_estimation_node
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES}
)

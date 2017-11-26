cmake_minimum_required(VERSION 2.8.3)
project(planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package()

# Add other local cpp files here
set(SOURCES
  src/pose.cpp
  src/map.cpp
  src/graph.cpp
  src/path.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning_node src/planning_node.cpp ${SOURCES})
add_executable(control_node src/control_node.cpp ${SOURCES})

target_link_libraries(planning_node
  ${catkin_LIBRARIES}
)
target_link_libraries(control_node
  ${catkin_LIBRARIES}
)

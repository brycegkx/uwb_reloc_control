cmake_minimum_required(VERSION 2.8.3)
project(uwb_reloc_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  uwb_reloc_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS uwb_reloc_msgs
)


include_directories(  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(uwb_reloc_control_node src/uwb_reloc_ctrl_v1.cpp)
add_executable(uwb_reloc_visualization_node src/uwb_reloc_viz_v1.cpp)

target_link_libraries(uwb_reloc_control_node ${catkin_LIBRARIES})
target_link_libraries(uwb_reloc_visualization_node ${catkin_LIBRARIES})

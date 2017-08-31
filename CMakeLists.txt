cmake_minimum_required(VERSION 2.8.3)
project(uwb_reloc_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  uwb_reloc_msgs
  tf
  mavros_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS uwb_reloc_msgs
)

set(csv_formats_inc include/csv_format/)
set(csv_parse_engine_inc include/csv_parser/)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${csv_formats_inc} ${csv_parse_engine_inc})

add_executable(uwb_reloc_control_node src/uwb_reloc_ctrl_v1.cpp)
add_executable(uwb_reloc_control_node_v2 src/uwb_reloc_ctrl_v2.cpp)
add_executable(uwb_reloc_visualization_node src/uwb_reloc_viz_v1.cpp)
add_executable(msg_translator_node src/msg_translator_v1.cpp)
add_executable(csv_parser src/csv_parser.cpp)

target_link_libraries(uwb_reloc_control_node ${catkin_LIBRARIES})
target_link_libraries(uwb_reloc_control_node_v2 ${catkin_LIBRARIES})
target_link_libraries(uwb_reloc_visualization_node ${catkin_LIBRARIES})
target_link_libraries(msg_translator_node ${catkin_LIBRARIES})
target_link_libraries(csv_parser ${catkin_LIBRARIES})

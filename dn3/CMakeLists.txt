cmake_minimum_required(VERSION 2.8.3)
project(dn3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(five_goals src/5_goals.cpp)
target_link_libraries(five_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


cmake_minimum_required(VERSION 2.8.3)
project(sl_visn)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	sensor_msgs
	geometry_msgs
	message_filters
	tf
)

catkin_python_setup()
catkin_package()
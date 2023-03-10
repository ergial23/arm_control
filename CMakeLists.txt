cmake_minimum_required(VERSION 3.0.2)
project(armpi)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf2
  tf2_ros
  tf_conversions
  robp_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	rospy 
	message_runtime 
 	tf2
	tf2_ros
	tf_conversions	
	robp_msgs
  	
	
)

catkin_install_python(PROGRAMS 
	scripts/ik_controller.py
	
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

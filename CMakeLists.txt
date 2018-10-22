cmake_minimum_required(VERSION 2.8.3)
project(my_rover3)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg mavros_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs mavros_msgs)

catkin_install_python(PROGRAMS
	scripts/my_rover3.py
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

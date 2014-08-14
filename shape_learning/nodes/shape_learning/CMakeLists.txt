cmake_minimum_required(VERSION 2.8.3)
project(shape_learning)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
## Mark executable scripts for installation
#install(PROGRAMS
# nodes/learning_demo.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
 
## Mark include directory for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
 )

## Mark datasets for installation
install(DIRECTORY letter_model_datasets/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/letter_model_datasets
 )

# \file	CMakeLists.txt
# \author	Alexi Demers <alexidemers@gmail.com>
# \date	02/12/2015
# \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>

cmake_minimum_required(VERSION 2.8.3)

project(provider_can)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug CACHE STRING
			"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
			FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")

set( provider_can_SRC_DIR "src" )

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_library(kvaser_LIBRARIES canlib /opt/linuxcan/canlib)

# Declare a catkin package
catkin_package(
		INCLUDE_DIRS ${provider_imu_SRC_DIR}
		LIBRARIES
		CATKIN_DEPENDS roscpp
)

#===============================================================================
# I N C L U D E   F I L E S

# Get all the files of the projects.
file(GLOB_RECURSE provider_can_FILES "${provider_can_SRC_DIR}/*.cc"
		"${provider_can_SRC_DIR}/*.h")

# Include all the directories where source can be found.
include_directories(
		${provider_can_SRC_DIR}
		${catkin_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

# Creating the ROS Node
add_executable(${PROJECT_NAME}_node ${provider_can_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${kvaser_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node provider_can_generate_messages_cpp)

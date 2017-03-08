set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_core)

find_package(catkin REQUIRED COMPONENTS roscpp tf nav_msgs message_generation cnbiros_messages cnbiros_services cv_bridge grid_map_ros)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_messages_INCLUDE_DIRS} 
		    ${Eigen3_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
	${Eigen3_INCLUDE_DIRS} 
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  	cnbiros_messages
	cnbiros_services
  DEPENDS
  	Eigen3
)

add_library(cnbiros_core 
			src/RosInterface.cpp
			src/GridMapTool.cpp
			src/RobotBase.cpp
			src/Odometry.cpp
			src/Sensor.cpp
			src/KinectScan.cpp
			src/Fusion.cpp
			src/Navigation.cpp
			src/ForceField.cpp)
target_link_libraries(cnbiros_core ${catkin_LIBRARIES} ${Eigen3_LIBRARY})

# Kinectscan node
add_executable(kinectscan_node_base src/kinectscan_node_base.cpp)
target_link_libraries(kinectscan_node_base cnbiros_core ${Eigen3_LIBRARY})

# Test ros interface
add_executable(test_ros_interface test/test_ros_interface.cpp)
target_link_libraries(test_ros_interface cnbiros_core)

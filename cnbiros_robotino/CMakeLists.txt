set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_robotino)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cnbiros_core)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)

find_package(RobotinoAPI2)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_core_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  	cnbiros_core
)

add_library(cnbiros_robotino 
			#src/RobotinoMotors.cpp
			src/Communication.cpp
			#src/RobotinoOdometry.cpp
			src/Infrared.cpp)
target_link_libraries(cnbiros_robotino ${catkin_LIBRARIES} cnbiros_core ${REC_ROBOTINO_API2_LIBRARY})

# Robotino motors
#add_executable(robotino_node_motors bin/robotino_node_motors.cpp)
#target_link_libraries(robotino_node_motors cnbiros_robotino ${Eigen3_LIBRARY})

# Infrared node
add_executable(robotino_node_infrared src/robotino_node_infrared.cpp)
target_link_libraries(robotino_node_infrared cnbiros_robotino)

## Odometry node
#add_executable(robotino_node_odometry src/robotino_node_odometry.cpp)
#target_link_libraries(robotino_node_odometry cnbiros_robotino ${Eigen3_LIBRARY})



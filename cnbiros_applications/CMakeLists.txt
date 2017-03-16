set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_applications)

find_package(catkin REQUIRED COMPONENTS roscpp cnbiros_core cnbiros_robotino cnbiros_messages cnbiros_services)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_messages_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
	${Eigen3_INCLUDE_DIRS} 
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	cnbiros_core
	cnbiros_robotino
  	cnbiros_messages
	cnbiros_services
)



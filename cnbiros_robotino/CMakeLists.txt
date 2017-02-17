set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_robotino)


find_package(catkin REQUIRED COMPONENTS roscpp cnbiros_core cv_bridge grid_map_ros)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH /usr/local/robotino/api2/cmake)

find_package(RobotinoAPI2)


include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_core_INCLUDE_DIRS} 
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
  	cnbiros_core
  DEPENDS
  	Eigen3
)

add_library(cnbiros_robotino 
			src/Robotino.cpp
			src/RobotinoCom.cpp
			src/RobotinoInfrared.cpp)
target_link_libraries(cnbiros_robotino ${catkin_LIBRARIES} cnbiros_core ${REC_ROBOTINO_API2_LIBRARY} ${Eigen3_LIBRARY} )

# Robotino node
add_executable(robotino_node
			   src/robotino_node.cpp)
target_link_libraries(robotino_node cnbiros_robotino ${Eigen3_LIBRARY})

# Test executables
add_executable(robotinotest
			   test/robotinotest.cpp)
target_link_libraries(robotinotest cnbiros_robotino)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_forcefield)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cnbiros_core grid_map_ros)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

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

add_library(cnbiros_forcefield 
			src/Actors.cpp)
target_link_libraries(cnbiros_forcefield
					  cnbiros_core 
					  ${catkin_LIBRARIES})

# Attractors node
#add_executable(forcefield_node_attractors src/forcefield_node_attractors.cpp)
#target_link_libraries(forcefield_node_attractors cnbiros_forcefield)

add_executable(forcefield_node_repellors src/forcefield_node_repellors.cpp)
target_link_libraries(forcefield_node_repellors cnbiros_forcefield)


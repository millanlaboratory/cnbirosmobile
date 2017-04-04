set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_forcefield)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cnbiros_core cnbiros_bci grid_map_ros)
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
	cnbiros_bci
)

add_library(cnbiros_forcefield 
			src/Actors.cpp
			src/DiscreteControl.cpp
			src/ContinuousControl.cpp)
target_link_libraries(cnbiros_forcefield
					  cnbiros_core 
					  ${catkin_LIBRARIES})

# BCI discrete control
add_executable(forcefield_bci_discrete src/forcefield_bci_discrete.cpp)
target_link_libraries(forcefield_bci_discrete cnbiros_forcefield)

# BCI continuous control
add_executable(forcefield_bci_continuous src/forcefield_bci_continuous.cpp)
target_link_libraries(forcefield_bci_continuous cnbiros_forcefield)

# Fusion actors node
add_executable(forcefield_fusion_actors src/forcefield_fusion_actors.cpp)
target_link_libraries(forcefield_fusion_actors cnbiros_forcefield)


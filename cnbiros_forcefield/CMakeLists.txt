set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_forcefield)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cnbiros_core cnbiros_bci grid_map_ros cnbiros_actions)
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
	cnbiros_actions
)

add_library(cnbiros_forcefield 
			src/Actors.cpp
			src/DiscreteControl.cpp
			src/ContinuousControl.cpp
			src/Dynamics.cpp)
target_link_libraries(cnbiros_forcefield
					  cnbiros_core 
					  cnbiros_actions
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

# Dynamic navigation ndoe
add_executable(forcefield_dynamics src/forcefield_dynamics.cpp)
target_link_libraries(forcefield_dynamics cnbiros_forcefield)

# Forcefield action server
add_executable(forcefield_pose_action src/forcefield_pose_action.cpp)
target_link_libraries(forcefield_pose_action cnbiros_forcefield)


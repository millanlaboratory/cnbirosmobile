set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_bci)

find_package(catkin REQUIRED COMPONENTS roscpp tf nav_msgs message_generation cnbiros_core cnbiros_messages cnbiros_services cv_bridge grid_map_ros)
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
	cnbiros_core
  	cnbiros_messages
	cnbiros_services
  DEPENDS
  	Eigen3
)

#add_library(cnbiros_bci 
#			src/TobiId.cpp
#			src/BciInput.cpp)
#target_link_libraries(cnbiros_bci ${catkin_LIBRARIES} ${Eigen3_LIBRARY} 
#					  cnbiros_core tobicore tobiid tobiic cnbiloop cnbicore)
#
## TobiId node
#add_executable(tobiid_node_base src/tobiid_node_base.cpp)
#target_link_libraries(tobiid_node_base 	cnbiros_bci cnbiros_core tobicore 
#										tobiid 
#										cnbiloop 
#										cnbicore)
#
## BCI Input node
#add_executable(bci_node_base src/bci_node_base.cpp)
#target_link_libraries(bci_node_base cnbiros_bci ${Eigen3_LIBRARY})

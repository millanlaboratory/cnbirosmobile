set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_bci)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation cnbiros_core)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} include)

add_message_files(
  FILES
  TiDMessage.msg
  TiCClass.msg
  TiCClassifier.msg
  TiCMessage.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   cnbiros_bci
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
    message_runtime
	cnbiros_core
)

add_library(cnbiros_bci 
			src/CnbiInterface.cpp
			src/TiDProxy.cpp
			src/TiCProxy.cpp
			src/TobiIcTools.cpp
			src/TobiIdTools.cpp)
target_link_libraries(cnbiros_bci ${catkin_LIBRARIES} 
					  cnbiros_core tobicore tobiid tobiic cnbiloop cnbicore)

# Main executables for TiC and TiD
add_executable(tobi_interface_c src/tobi_interface_c.cpp)
target_link_libraries(tobi_interface_c cnbiros_bci)

add_executable(tobi_interface_d src/tobi_interface_d.cpp)
target_link_libraries(tobi_interface_d cnbiros_bci)

# Examples
add_executable(cnbiinterface examples/cnbiinterface.cpp)
target_link_libraries(cnbiinterface cnbiros_bci)

add_executable(tobiid examples/tobiid.cpp)
target_link_libraries(tobiid cnbiros_bci)

add_executable(tobiic examples/tobiic.cpp)
target_link_libraries(tobiic cnbiros_bci)

add_executable(tobiic_sender examples/tobiic_sender.cpp)
target_link_libraries(tobiic_sender cnbiros_bci)

# cl_ticsender -> To be moved to cnbitk
add_executable(cl_ticsender src/cl_ticsender.cpp)
target_link_libraries(cl_ticsender cnbiros_bci)

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

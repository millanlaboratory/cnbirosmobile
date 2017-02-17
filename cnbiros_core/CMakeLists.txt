set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_core)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation cnbiros_messages)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} ${cnbiros_messages_INCLUDE_DIRS} include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  cnbiros_messages
)

add_library(cnbiros_core 
			src/Robot.cpp
			src/RobotFactory.cpp
			src/Sensor.cpp
			src/SensorFactory.cpp
			src/Infrared.cpp)
target_link_libraries(cnbiros_core ${catkin_LIBRARIES})

add_executable(sensors
			   test/sensors.cpp)
target_link_libraries(sensors cnbiros_core)



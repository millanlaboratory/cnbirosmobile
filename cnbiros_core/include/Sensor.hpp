#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#define SENSOR_BUFFER_SIZE 1000

namespace cnbiros {
	namespace core {

class Sensor {

	public:
		Sensor(std::string name);
		virtual ~Sensor(void);
		
		void Register(ros::NodeHandle* node);
		bool IsRegistered(void);
		void Advertise(std::string topic);


		virtual void Process(void) = 0;
	
	protected:
		void SetGridParameters(std::string layer, std::string frameid);
		void SetGridDimensions(float xdim, float ydim, float resolution);

	public:
		enum Type {
			Infrared,
			Sonar,
			Kinect,
			Camera
		};

	protected:
		unsigned int 	frequency_;
		
		// ros related members
		ros::NodeHandle* rosnode_;
		std::string 	 rostopic_;
		std::string 	 rosname_;
		ros::Publisher 	 rospub_;
		ros::Rate* 		 rosrate_;

		// grid related members
		float grid_xdim_;
		float grid_ydim_;
		float grid_resolution_;
		std::string grid_base_layer_;
		std::string grid_base_frameid_;
		grid_map::GridMap 		grid_;
		grid_map_msgs::GridMap 	msg_;

};
	}
}

#endif

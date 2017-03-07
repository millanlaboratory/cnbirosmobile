#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Flags.hpp"
#include "RosInterface.hpp"

namespace cnbiros {
	namespace core {

class Odometry : public RosInterface {

	public:
		Odometry(ros::NodeHandle* node);
		virtual ~Odometry(void);

		void AdvertiseOn(std::string topic);	
		void Reset(nav_msgs::Odometry& msg);
		nav_msgs::Odometry ConvertToMessage(float x, float y, float z, float omega,
							  		  	    float vx, float vy, float vz, float vomega, 
									  	    unsigned int sequence);

};


	}
}


#endif

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
		virtual void Run(void) = 0;
	protected:
		virtual void compute_odometry(float x, float y, float z, 
							  		  float vx, float vy, float vz, 
							  		  float vomega, unsigned int sequence);

		//virtual void compute_tf(float x, float y, float z, float omega);
	protected:

		nav_msgs::Odometry 				rosodom_msg_;
};


	}
}


#endif

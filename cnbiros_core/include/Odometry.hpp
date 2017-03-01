#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class Odometry {

	public:
		Odometry(ros::NodeHandle* node, 
				 float frequency,
				 std::string frameid,
				 std::string child_frameid);
		virtual ~Odometry(void);

		
		virtual void Run(void) = 0;
	protected:
		virtual void compute_tf(float x, float y, float z, float omega);
		virtual void compute_odometry(float x, float y, float z, 
							  		  float vx, float vy, float vz, 
							  		  float vomega, unsigned int sequence);

	protected:
		float frequency_;
		std::string frame_id_;
		std::string child_frame_id_;

		nav_msgs::Odometry 				rosodom_msg_;
		geometry_msgs::Quaternion 		rosodom_quat_;
		geometry_msgs::TransformStamped rosodom_tf_;

		ros::NodeHandle* 		 rosnode_;
		ros::Rate* 				 rosrate_;
		ros::Publisher 			 rospub_;
		tf::TransformBroadcaster rostf_;


};


	}
}


#endif

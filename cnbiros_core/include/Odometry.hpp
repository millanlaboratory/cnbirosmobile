#ifndef CNBIROS_CORE_ODOMETRY_HPP
#define CNBIROS_CORE_ODOMETRY_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "cnbiros_services/OdometryReset.h"

namespace cnbiros {
	namespace core {

class Odometry : public RosInterface {

	public:
		Odometry(ros::NodeHandle* node, std::string name);
		virtual ~Odometry(void);

	protected:
		
		virtual void onRunning(void) = 0;
		virtual void onReset(void) = 0;

		void reset_message(void);
		void set_message(float x, float y, float z, float omega,
						 float vx, float vy, float vz, float vomega, 
						 unsigned int sequence);

	private:
		 virtual bool on_odometry_reset_(cnbiros_services::OdometryReset::Request& req,
										 cnbiros_services::OdometryReset::Response& res);

	protected:
		nav_msgs::Odometry 	rosodom_msg_;
		ros::ServiceServer	rossrv_reset_;
};


	}
}


#endif

#ifndef CNBIROS_FORCEFIELD_DISCRETE_HPP
#define CNBIROS_FORCEFIELD_DISCRETE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_bci/TiDMessage.h"
#include "cnbiros_bci/TobiIdTools.hpp"
#include "cnbiros_services/Reset.h"


namespace cnbiros {
	namespace forcefield {

class DiscreteControl : public core::RosInterface {

	public:
		DiscreteControl(std::string name = "discrete_control");
		~DiscreteControl(void);

		void AddSource(std::string topic);
		void SetRadius(float radius);
		void SetFilter(std::string pipe, int family = -1);	
		void SetCommand(std::string event, float angle);
		bool CheckMessage(const cnbiros_bci::TiDMessage& msg);
		void Reset(void);

		void onRunning(void);
		bool TargetReached(void);
	private:
		void onReceived(const cnbiros_bci::TiDMessage& msg);
		void onReceivedOdometry(const nav_msgs::Odometry& msg);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);

	private:
		std::string  topic_;

		bool 			flt_is_set_;
		std::string 	flt_pipe_;
		int 			flt_family_;
		float 			radius_;

		bool 				is_target_set_;
		float 				target_;
		nav_msgs::Odometry 	odometry_;
		
		std::map<std::string, float> 	cmd_angles_;
		sensor_msgs::PointCloud 		data_;

		ros::ServiceServer 	rossrv_reset_;
};

	}
}




#endif

#ifndef CNBIROS_FORCEFIELD_CONTINUOUS_HPP
#define CNBIROS_FORCEFIELD_CONTINUOUS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_bci/TiCMessage.h"
#include "cnbiros_bci/TobiIcTools.hpp"
#include "cnbiros_services/Reset.h"


namespace cnbiros {
	namespace forcefield {

class ContinuousControl : public core::RosInterface {

	public:
		ContinuousControl(std::string name = "continuous_control");
		~ContinuousControl(void);

		void AddSource(std::string topic);

		void SetRadius(float radius);
		void SetFilter(std::string pipe, std::string name, std::string label);

		bool CheckMessage(const cnbiros_bci::TiCMessage& msg);
		void Reset(void);

		void onRunning(void);
	private:
		void onReceived(const cnbiros_bci::TiCMessage& msg);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);

	private:
		std::string  topic_;

		bool 			flt_is_set_;
		std::string 	flt_pipe_;
		std::string 	flt_name_;
		std::string		flt_label_;

		float radius_;
		sensor_msgs::PointCloud 		data_;

		ros::ServiceServer 	rossrv_reset_;
};

	}
}

#endif


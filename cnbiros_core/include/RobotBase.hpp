#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "Flags.hpp"

namespace cnbiros {
	namespace core {

class RobotBase {
	public:
		RobotBase(unsigned int type, std::string name="", std::string id="");
		virtual ~RobotBase(void);

		unsigned int GetType(void);
		void SetType(unsigned int type);
		
		std::string GetName(void);
		void SetName(std::string name);

		std::string GetIdentifier(void);
		void SetIdentifier(std::string id);
		
		void Register(ros::NodeHandle* node);
		bool IsRegistered(void);
		void Subscribe(std::string topic = CNBIROS_TOPIC_VELOCITY);

		virtual void Dump(void);
		
		virtual void Run(void) = 0;

	protected:
		virtual void velocityCallback(const geometry_msgs::Twist& msg) = 0;

	public:
		enum Type {
			Robotino,
			Wheelchair,
			Drone
		};

	protected:
		unsigned int 	type_;
		std::string 	name_;
		std::string 	identifier_;
		unsigned int 	frequency_;

		// ros related members
		std::string 	 topic_;
		ros::NodeHandle* rosnode_;
		ros::Subscriber  rossub_;
		ros::Rate* 		 rosrate_;


};

	}
}

#endif

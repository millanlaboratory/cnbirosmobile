#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <ros/ros.h>
#include "cnbiros_messages/RobotVelocity.h"


namespace cnbiros {
	namespace core {

class Robot {
	public:
		Robot(unsigned int type, std::string name, std::string id);
		virtual ~Robot(void){};

		unsigned int GetType(void);
		
		std::string GetName(void);
		void SetName(std::string name);

		std::string GetIdentifier(void);
		void SetIdentifier(std::string id);

		virtual void Subscribe(ros::NodeHandle* node, std::string topic);

		virtual void Dump(void);
		

		// Just an example of methods
		virtual int Run(void) = 0;
		virtual int Stop(void) = 0;

	protected:
		virtual void velocityCallback(const cnbiros_messages::RobotVelocity& msg) = 0;

	public:
		enum Type {
			Robotino,
			Wheelchair,
			Drone
		};

	protected:
		unsigned int type_;
		std::string name_;
		std::string identifier_;
		//bool isconnected_;

		// ros related members
		ros::Subscriber rossub_;

};

	}
}

#endif

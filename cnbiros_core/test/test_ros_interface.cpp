#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "RosInterface.hpp"

using namespace cnbiros::core;

void callback_msg_string(const std_msgs::String& msg) {
	printf("[FunctionCallback] Received std_msg::String\n");
}

void callback_msg_float(const std_msgs::Float32& msg) {
	printf("[FunctionCallback] Received std_msg::Float\n");
}

class DerivedInterface : public RosInterface {
	public:
		DerivedInterface(ros::NodeHandle* node) : RosInterface(node) {};
		~DerivedInterface(void) {};

		void callback_object_msg_string(const std_msgs::String& msg) {
			printf("[ObjectCallback] Received std_msg::String\n");
		}
		
		void callback_object_msg_float(const std_msgs::Float32& msg) {
			printf("[ObjectCallback] Received std_msg::Float\n");
		}

		void onRunning(void) {};

};

int main (int argc, char** argv) {


	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
								   ros::console::levels::Debug);
	
	ros::init(argc, argv, "test_interface");

	ros::NodeHandle node("~");

	DerivedInterface iface_d(&node);
	iface_d.SetSubscriber("/test_topic", &DerivedInterface::callback_object_msg_string, &iface_d);
	iface_d.SetSubscriber("/test_topic2", &DerivedInterface::callback_object_msg_float, &iface_d);
	iface_d.SetSubscriber("/test_topic3", callback_msg_string);
	iface_d.SetSubscriber("/test_topic4", callback_msg_float);
	iface_d.SetPublisher<std_msgs::Float32>("/test_topic3");
	
	ros::Subscriber* gsub = iface_d.GetSubscriber("/test_topic2");
	
	ros::Rate r(1);
	std_msgs::Float32 msg;

	while(node.ok()) {
		
		msg.data = 10.0f;
		iface_d.Publish(msg); 
		ROS_DEBUG_THROTTLE(5, "[test] - Number of publishers for %s: %d\n", "/test_topic2", gsub->getNumPublishers());


		r.sleep();
		ros::spinOnce();
	}













	return 0;
}

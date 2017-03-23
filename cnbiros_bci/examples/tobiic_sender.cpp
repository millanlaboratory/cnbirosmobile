#include <ros/ros.h>

#include "RosInterface.hpp"
#include "CnbiInterface.hpp"
#include "TiCProxy.hpp"
#include "cnbiros_bci/TiCMessage.h"

class RosSender : public cnbiros::core::RosInterface {
	public:
		RosSender(void) : cnbiros::core::RosInterface("rossender_tobiic") {
			this->SetPublisher<cnbiros_bci::TiCMessage>("/tic_to_cnbiloop0");

			this->class1.label = "class_test1";
			this->class2.label = "class_test2";
			this->class3.label = "class_test3";
			
			this->class1.value = 0.0f;
			this->class2.value = 0.0f;
			this->class3.value = 0.0f;

			this->classifier1.name = "classifier1";
			this->classifier1.description = "Description classifier 1";
			this->classifier1.vtype = 0;
			this->classifier1.ltype = 0;
			this->classifier1.classes.push_back(this->class1);
			this->classifier1.classes.push_back(this->class2);
			
			this->classifier2.name = "classifier2";
			this->classifier2.description = "Description classifier 2";
			this->classifier2.vtype = 0;
			this->classifier2.ltype = 0;
			this->classifier2.classes.push_back(this->class3);
		}

		~RosSender(void) {};


		void onRunning(void) {
			
			cnbiros_bci::TiCMessage 	msg;
			msg.pipe = "/ctrl13";
			
			this->classifier1.classes.clear();
			this->classifier2.classes.clear();

			this->class1.value = this->class1.value + 0.1f;
			this->class2.value = this->class2.value - 0.1f;
			this->class3.value = 1.0f;

			this->classifier1.classes.clear();
			this->classifier1.classes.push_back(this->class1);
			this->classifier1.classes.push_back(this->class2);
			
			this->classifier2.classes.clear();
			this->classifier2.classes.push_back(this->class3);

			msg.classifiers.clear();
			msg.classifiers.push_back(this->classifier1);
			msg.classifiers.push_back(this->classifier2);

			this->Publish("/tic_to_cnbiloop0", msg);
		}

	private:

		cnbiros_bci::TiCClassifier classifier1;
		cnbiros_bci::TiCClassifier classifier2;
		cnbiros_bci::TiCClass 	   class1;
		cnbiros_bci::TiCClass 	   class2;
		cnbiros_bci::TiCClass 	   class3;

};

int main(int argc, char** argv) {


	// ROS initialization
	ros::init(argc, argv, "tobiicsender_node_example");
	ros::NodeHandle node("~");
	ros::Rate rate(10);


	// TobiIc message sender
	RosSender sender;


	sender.Run();

	return 0;
}

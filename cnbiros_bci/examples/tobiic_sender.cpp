#include <ros/ros.h>

#include "RosInterface.hpp"
#include "CnbiInterface.hpp"
#include "TobiIc.hpp"
#include "cnbiros_bci/TobiIc.h"

class RosSender : public cnbiros::core::RosInterface {
	public:
		RosSender(void) : cnbiros::core::RosInterface("rossender_tobiic") {
			this->SetPublisher<cnbiros_bci::TobiIc>("/test");

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
			this->classifier1.IcClasses.push_back(this->class1);
			this->classifier1.IcClasses.push_back(this->class2);
			
			this->classifier2.name = "classifier2";
			this->classifier2.description = "Description classifier 2";
			this->classifier2.vtype = 0;
			this->classifier2.ltype = 0;
			this->classifier2.IcClasses.push_back(this->class3);
		}

		~RosSender(void) {};


		void onRunning(void) {
			
			cnbiros_bci::TobiIc 	msg;
			msg.pipe = "/ctrl1";
			
			this->classifier1.IcClasses.clear();
			this->classifier2.IcClasses.clear();

			this->class1.value = this->class1.value + 0.1f;
			this->class2.value = this->class2.value - 0.1f;
			this->class3.value = 1.0f;

			this->classifier1.IcClasses.clear();
			this->classifier1.IcClasses.push_back(this->class1);
			this->classifier1.IcClasses.push_back(this->class2);
			
			this->classifier2.IcClasses.clear();
			this->classifier2.IcClasses.push_back(this->class3);

			msg.IcClassifiers.clear();
			msg.IcClassifiers.push_back(this->classifier1);
			msg.IcClassifiers.push_back(this->classifier2);

			this->Publish("/test", msg);
		}

	private:

		cnbiros_bci::TobiIcClassifier classifier1;
		cnbiros_bci::TobiIcClassifier classifier2;
		cnbiros_bci::TobiIcClass 	  class1;
		cnbiros_bci::TobiIcClass 	  class2;
		cnbiros_bci::TobiIcClass 	  class3;

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

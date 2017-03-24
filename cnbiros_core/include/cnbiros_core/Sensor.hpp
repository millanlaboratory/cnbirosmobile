#ifndef CNBIROS_CORE_SENSOR_HPP 
#define CNBIROS_CORE_SENSOR_HPP

#include <ros/ros.h> 

#include "cnbiros_core/Flags.hpp" 
#include "cnbiros_core/RosInterface.hpp" 
#include "cnbiros_services/Reset.h"

namespace cnbiros { 
	namespace core {

//* Sensor class
/** 
 * Sensor() class represents the base class for any kind of sensors developed
 * in the cnbiros_mobile framework. It is a pure abstract class directly
 * derived from RosInterface(). It provides attributes and it implements
 * methods common to different types of sensors. In particular, it has a
 * grid_map::GridMap member that corresponds to the common format used to share
 * sensor information between nodes in cnbiros_mobile framework.
 * 
 * <b>Behaviour:</b> \n 
 * The basic assumption for the Sensor() class is that it acquires data from an
 * hardware sensor and after some processing it publishes the data in the
 * grid_map format. The grid map is initialized with default size and
 * resolution and a layer is created with the name of the sensor.  Once
 * instanciated the sensor advertise on topic <i>/sensor_$NAME</i> with $NAME
 * equals to the provided name set in the constructor.  While running, the
 * sensor publishes on the aforementioned topic a grid_map_msgs. If a different
 * publishing topic is required, please consider ros remap tool instead of
 * changing manually the topic name.
 * 
 * <b>Basic methods and members:</b> \n 
 * <ul>
 * <li> Sensor(ros::NodeHandle* node, std::string name): 
 * Class constructor needs a pointer to the ros node handler and name of the
 * sensor. This name is set automatically during instanciation.  
 * <li> #rosgrid_: 
 * a grid_map::GridMap member to be used in the derived classes.  
 * <li> onStop(), onStart(): 
 * once the sensor starts or stops, its own grid map is reset to 0.  
 * <li> onRunning(): 
 * pure virtual method to be implemented in the derived class.
 * </ul>
 * 
 * <b>Services:</b> \n 
 * By default, a Sensor() implements a ROS service to reset its own grid map.
 * The service is accessible via ROS methods on <i>~/gridmap_reset</i>
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 *
 */

template<class T>
class Sensor : public RosInterface {

	public:
		Sensor(std::string name);
		virtual ~Sensor(void);

		void Get(T& data);
		void Set(const T& data);
		virtual void Reset(void);

	protected:
		virtual void onStop(void);
		virtual void onStart(void);
		virtual void onRunning(void);

	private: 
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);

	protected: 
		std::string 			rostopic_;
		T						sensor_data_;

	private: 
		ros::ServiceServer 		rossrv_reset_;
		
};

template<class T>
Sensor<T>::Sensor(std::string name) : RosInterface(name) {

	// Sensor initialization
	this->rostopic_ = "/sensor_" + this->GetName();
	this->SetPublisher<T>(this->rostopic_);

	// Service for sensor reset
	this->rossrv_reset_ = this->advertiseService("sensor_reset", 
											&Sensor<T>::on_service_reset_, this);
}

template<class T>
Sensor<T>::~Sensor(void) {};

template<class T>
void Sensor<T>::Get(T& data) {
	data = this->sensor_data_;
}

template<class T>
void Sensor<T>::Set(const T& data) {
	this->sensor_data_ = data;
}

template<class T>
bool Sensor<T>::on_service_reset_(cnbiros_services::Reset::Request& req,
						  		   cnbiros_services::Reset::Response& res) {
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("Sensor %s requested to reset", this->GetName().c_str());
		
		if(this->IsStopped() == false) {
			this->Reset();
			this->Publish(this->rostopic_, this->sensor_data_);
			res.result = true;
		}
	}

	return res.result;
}

template<class T>
void Sensor<T>::onStop(void) {
	this->Reset();
	this->Publish(this->rostopic_, this->sensor_data_);
}

template<class T>
void Sensor<T>::onStart(void) {
	this->Reset();
	this->Publish(this->rostopic_, this->sensor_data_);
}

template<class T>
void Sensor<T>::onRunning(void) {}


template<class T>
void Sensor<T>::Reset(void) {}

	} 
}

#endif

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
 * \brief Sensor class is the base class for different kind of sensors.
 *
 * Sensor() class represents the base class for any kind of sensors developed in
 * the cnbiros_mobile framework. It is a template class able to deal with
 * generic message types and it directly derived from RosInterface(). It
 * provides attributes and it implements methods common to different types of
 * sensors. 
 *  
 * \par General description:
 * The basic assumption for the Sensor() class is that it acquires data from an
 * hardware sensor and after some processing it publishes the data on the given
 * topic. Once instanciated the sensor advertise on topic <i>/sensor_$NAME</i>
 * with $NAME equals to the provided name set in the constructor.  While
 * running, the sensor publishes on the aforementioned topic a specific message
 * type. If a different publishing topic is required, please consider ros remap
 * tool instead of changing manually the topic name.
 * 
 * \par Basic methods and members:
 * - Sensor(std::string name): 
 * Class constructor needs the name of the sensor. This name is set
 * automatically during instanciation and can be retrieved by the method
 * GetName();
 * - GetData(), SetData():
 * Templates metod to set or retrieve the data of the sensor
 * - onRunning(): 
 * Empty virtual function to be implemented in the derived classes according to
 * the hardware and to the type of message
 * - Reset():
 * Empty virtual function to be implemented in the derived classes according to
 * the hardware and to the type of message
 * - onStop(), onStart(): 
 * Once the sensor starts or stops, the virtual method Reset() is called.  
 * 
 * \par Services:
 * By default, a Sensor() implements a ROS service to reset its own sensor data.
 * The service is accessible via ROS methods on <i>reset_sensor</i>
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 *
 */
template<class T>
class Sensor : public RosInterface {

	public:

		/*! \brief Constructor
		 *
		 * \param name 	Name of the sensor
		 */
		Sensor(std::string name);

		/*! \brief Destructor
		 */
		virtual ~Sensor(void);

		/*! \brief Get current sensor data
		 *
		 * \param[out] 	data 	Reference to the data
		 */
		void GetData(T& data);

		/*! \brief Set sensor data
		 * \param 	data 	Reference to the data to be set
		 */
		void SetData(const T& data);

		/*! \brief Empty virtual function to reset sensor data accordingly to
		 * the hardware and to the type of data
		 */
		virtual void Reset(void);

	protected:
		/*! \brief Callback called when the sensor stops
		 */
		virtual void onStop(void);
		
		/*! \brief Callback called when the sensor starts
		 */
		virtual void onStart(void);

		/*! \brief Callback called when the sensor is running
		 */
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
	this->rossrv_reset_ = this->advertiseService("reset_sensor", 
											&Sensor<T>::on_service_reset_, this);
}

template<class T>
Sensor<T>::~Sensor(void) {};

template<class T>
void Sensor<T>::GetData(T& data) {
	data = this->sensor_data_;
}

template<class T>
void Sensor<T>::SetData(const T& data) {
	this->sensor_data_ = data;
}

template<class T>
bool Sensor<T>::on_service_reset_(cnbiros_services::Reset::Request& req,
						  		   cnbiros_services::Reset::Response& res) {
	res.result = false;
	if(req.reset == true) {
		ROS_INFO("Sensor %s has been requested to reset", this->GetName().c_str());
		
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
	ROS_INFO("%s has been required to stop", this->GetName().c_str());
	this->Reset();
	this->Publish(this->rostopic_, this->sensor_data_);
}

template<class T>
void Sensor<T>::onStart(void) {
	ROS_INFO("%s has been required to start", this->GetName().c_str());
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

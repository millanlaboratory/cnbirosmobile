#ifndef CNBIROS_CORE_ROSINTERFACE_HPP
#define CNBIROS_CORE_ROSINTERFACE_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "cnbiros_services/RosInterfaceState.h"
#include "Flags.hpp"

namespace cnbiros {
	namespace core {

enum RosInterfaceState{Start, Stop, Resume};


class RosInterface {

	public:
		//! Constructor
		RosInterface(ros::NodeHandle* node);

		//! Destructor
		virtual ~RosInterface(void);

		//! Set interface name 
		//! \param name Name of the interface
		void SetName(std::string name);

		//! Get interface name
		//! \return The interface name
		std::string GetName(void);

		//! Set interface frequency
		//! \param frequency Frequency of the interface
		void SetFrequency(float frequency);
		
		//! Get current interface frequency
		//! \return The interface frequency
		float GetFrequency(void);

		//! Get an interface subscriber
		/*! \param topic subscriber topic
		 *  \return Pointer to the subscriber
		 */
		ros::Subscriber* GetSubscriber(std::string topic);

		//! Set a new interface subscriber (with external function as callback)
		/*! \param topic 	Name of the subscribed topic
		 *  \param fp 		Callback function to be called when a message
		 *  				arrives on the subscribed topic
		 */
		template<class M>
		void SetSubscriber(std::string topic, void(*fp)(M));
		
		//! Set a new interface subscriber (with class method as callback)
		/*! \param topic 	Name of the subscribed topic
		 *  \param fp 		Callback function to be called when a message
		 *  				arrives on the subscribed topic
		 *  \param obj		Pointer to the object where the callback is
		 *  				istantiated
		 */
		template<class M, class T>
		void SetSubscriber(std::string topic, void(T::*fp)(M), T* obj);

		//! Set a new interface publisher
		/*! \param topic 	Name of the advertise topic
		 */
		template<class M>
		void SetPublisher(std::string topic);

		//! Publish message on advertise topic 
		/*!  \param msg 		Message to be published
		 */
		template<class M>
		void Publish(M& msg);

		//! Main run method
		void Run(void);

		//! Set stop flag to true
		//! \sa Resume(), IsStopped()
		void Stop(void);

		//! Set stop flag to false
		//! \sa Stop(), IsStopped()
		void Resume(void);

		//! Check if the stop flag is wheter true or false. This method can be
		//! used in the main node loop 
		//! \return Stop flag
		//! \sa Stop(), Resume()
		bool IsStopped(void);

		void SetParentFrame(std::string frameid);
		void SetChildFrame(std::string frameid);
		void SetTransformMessage(tf::Vector3 translation, float yaw);
		void SetTransformMessage(tf::Vector3 translation, geometry_msgs::Quaternion quaternion);
		void TransformPoint(std::string frame, geometry_msgs::PointStamped& in,
				            geometry_msgs::PointStamped& out);

		std::string GetParentFrame(void);
		std::string GetChildFrame(void);
		geometry_msgs::TransformStamped GetTransformMessage(void);
	
	protected:
		//! Callback to be executed while the interface is running
		//! \sa Run()
		virtual void onRunning(void);

		virtual void onStop(void) {};
		virtual void onStart(void) {};

	private:
		void SendTransform(geometry_msgs::TransformStamped msg);
		
		bool on_rosinterface_service_(cnbiros_services::RosInterfaceState::Request &req,
									  cnbiros_services::RosInterfaceState::Response &res);

	protected:
		ros::NodeHandle* 			rosnode_; 				// <- private?
		ros::Rate* 					rosrate_; 				// <- private?
		ros::Publisher 				rospub_;
		std::map<std::string, ros::Subscriber> 	rossubs_;

	private:
		//! Generic interface members
		std::string 	name_;
		float 			frequency_;
		bool 			is_stopped_;

		//! Services related members
		ros::ServiceServer rossrv_state_;

		//! Frame related members
		std::string 	rosframe_child_;
		std::string 	rosframe_parent_;

		//! Transform related members
		tf::TransformBroadcaster 		rostf_broadcaster_; 
		tf::TransformListener 			rostf_listener_;
		geometry_msgs::TransformStamped rostf_msg_;
};


template<class M>
void RosInterface::SetSubscriber(std::string topic, void(*fp)(M)) {
	this->rossubs_[topic] = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, fp);
}

template<class M, class T>
void RosInterface::SetSubscriber(std::string topic, void(T::*fp)(M), T* obj) {
	this->rossubs_[topic] = this->rosnode_->subscribe(topic, CNBIROS_MESSAGES_BUFFER, fp, obj);
}

template<class M>
void RosInterface::SetPublisher(std::string topic) {
	this->rospub_ = this->rosnode_->advertise<M>(topic, CNBIROS_MESSAGES_BUFFER);
}

template<class M>
void RosInterface::Publish(M& msg) {
	this->rospub_.publish(msg);
}


	}
}


#endif
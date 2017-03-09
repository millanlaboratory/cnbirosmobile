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

//* RosInterface class
/**
 *	RosInterface represents the base class to interface with ROS. 
 *	It wraps some common ros methods and hides their complexity 
 *	(i.e., subscribe and advertise topics).
 * 	
 *	It is an abstract class, so it can be instanciated only as derived class
 *	with explicitly implementation of the callback method onRunning().
 *
 *	An important attribute of the class is its name (accessible via SetName(),
 *	GetName()). This attribute is used to automatically advertise and subscribe
 *	topics in its derived classes (and possibly services).
 *
 * 	Once instantiated, a derived class of RosInterface start an internal loop
 * 	that spins on ROS subscribed topics and ROS services. While the
 * 	interface is running, specific operations can be defined in the onRunning()
 * 	virtual method.
 *
 * 	Two secondary callbacks are called when the interfaces is stopped or started
 * 	(onStop() and onStart() virtual methods, respectively). By default, these
 * 	callbacks are empty, but they can be customized in the derived classes.
 *
 * 	In addition, the RosInterface provides some method to deal with the ROS
 * 	Transformation. By default, if defined, a transformation is published
 * 	automatically in the interface loop.
 *
 * 	Finally, RosInterface starts a standard ROS service to control the state of
 * 	the interface (Start, Stop, Resume). The service is accessible via ROS
 * 	methods on "~/rosinterface_state".
 *
 */
class RosInterface {

	public:
		/*! \brief Constructor
		 *
		 * Constructor with pointer to ROS node handler
		 */
		RosInterface(ros::NodeHandle* node);

		//! \brief Destructor
		virtual ~RosInterface(void);

		/*! \brief Set interface name 
		 *
		 * The name of the interface will be used to automatically set most of
		 * the subscriber/publisher/services.
		 *
		 * \param name Name of the interface
		 */
		void SetName(std::string name);

		/*! \brief Get interface name
		 * \return The interface name
		 */
		std::string GetName(void);

		/*! \brief Set interface frequency 
		 *
		 * The frequency of the interface represents the rate at which the loop
		 * is cycling.
		 *
		 * By default the frequency rate of the interface is set at
		 * CNBIROS_NODE_FREQUENCY. The frequency can be changed also if the
		 * interface is already running.
		 *
		 * \param frequency Frequency of the interface
		 */
		void SetFrequency(float frequency);
		
		/*! \brief Get current interface frequency
		 * \return The interface frequency
		 */
		float GetFrequency(void);

		/*! \brief Get the interface subscriber on topic
		 *
		 * Returns a pointer of type ros::Subscriber related to required subscriber.
		 *
		 *  \param topic subscriber topic
		 *  \return Pointer to the subscriber
		 */
		ros::Subscriber* GetSubscriber(std::string topic);

		/*! \brief Set a new interface subscriber on topic 
		 *
		 * Add a new subscriber to the interface subscriber list. This method is
		 * used with external function as callback.
		 *
		 * \param topic 	Name of the subscribed topic
		 * \param fp 		Callback function to be called when a message
		 *  				arrives on the subscribed topic
		 */
		template<class M>
		void SetSubscriber(std::string topic, void(*fp)(M));
		
		/*! \brief Set a new interface subscriber on topic
		 * 	
		 * 	Add a new subscriber to the interface subscriber list. This method is
		 * 	used with a class method as callback.
		 * 	\param topic 	Name of the subscribed topic
		 *  \param fp 		Callback function to be called when a message
		 *  				arrives on the subscribed topic
		 *  \param obj		Pointer to the object where the callback is
		 *  				istantiated
		 */
		template<class M, class T>
		void SetSubscriber(std::string topic, void(T::*fp)(M), T* obj);

		/*! \brief Set a new interface publisher
		 *
		 *	Create a new publisher that advertises on the required topic.
		 *	By assumption, a RosInterface has only a publisher.
		 *
		 *  \param topic 	Name of the advertise topic
		 */
		template<class M>
		void SetPublisher(std::string topic);

		/*! \brief Publish message on advertise topic 
		 *
		 *   \param msg 		Message to be published
		 */
		template<class M>
		void Publish(M& msg);

		/*! \brief Main run method
		 * 	
		 * 	Method to start the interface
		 */ 
		void Run(void);

		/*! \brief Stop the interface 
		 * \sa Resume(), IsStopped()
		 */ 
		void Stop(void);

		/*! \brief Restart the interface 
		 * 	\sa Stop(), IsStopped()
		 */
		void Resume(void);

		/*! \brief Check if the interface is stopped
		 * \return Stop flag
		 * \sa Stop(), Resume()
		 */
		bool IsStopped(void);

		/*! \brief Set the parent frame for the interface
		 *
		 * Set the parent frame for the interface for transformation purposes
		 * \param frameid id of parent frame
		 */
		void SetParentFrame(std::string frameid);

		/*! \brief Set the child frame id for the interface
		 *
		 * Set the child frame id for the interface for transformation purposes
		 * \param frameid id of child frame
		 */
		void SetChildFrame(std::string frameid);

		/*! \brief Set the transformation of the interface
		 *
		 * Update the transformation of the interface, given a translation
		 * vector and the orientation with respect the z-axis
		 *
		 * \param translation 	Vector with translation in x-, y-, z-axis
		 * \param yaw 			Value of orientation with respect to z-axis
		 */
		void SetTransformMessage(tf::Vector3 translation, float yaw);
		
		/*! \brief Set the transformation of the interface
		 *
		 * Update the transformation of the interface, given a translation
		 * vector and a quaternion 
		 *
		 * \param translation 	Vector with translation in x-, y-, z-axis
		 * \param quaternion 	Quaternion
		 */
		void SetTransformMessage(tf::Vector3 translation, geometry_msgs::Quaternion quaternion);

		/*! \brief Points transformation with respect to the required frame
		 *
		 * Transforms an input point messages in the out message with respect to
		 * the given frame
		 * \param 		frame 	target frame for transformation
		 * \param[in] 	in 		Input points
		 * \param[out] out		Output points (transformed)
		 */
		void TransformPoint(std::string frame, geometry_msgs::PointStamped& in,
				            geometry_msgs::PointStamped& out);

		/*! \brief Get parent frame id
		 * \return Parent frame id
		 */
		std::string GetParentFrame(void);
		
		/*! \brief Get child frame id
		 * \return Child frame id
		 */
		std::string GetChildFrame(void);
		
		/*! \brief Convert transformation in message
		 *
		 * It converts the current RosInterface transformation to a geometry
		 * message.
		 * \return Converted transformation
		 */
		geometry_msgs::TransformStamped GetTransformMessage(void);
	
	protected:
		/*! Callback to be executed while the interface is running
		 * 
		 * This callback is called at every iteration of the interface. Can be
		 * instanciated and defined in the derived class.
		 * \sa Run()
		 */
		virtual void onRunning(void) = 0;

		/*! Callback to be executed when the interface stops
		 * 
		 * This callback is called when the interface stops. Can be
		 * instanciated and defined in the derived class.
		 * \sa Stop()
		 */
		virtual void onStop(void) {};
		
		/*! Callback to be executed when the interface starts
		 * 
		 * This callback is called when the interface starts. Can be
		 * instanciated and defined in the derived class.
		 * \sa Start()
		 */
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

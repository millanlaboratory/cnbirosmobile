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
 * \brief Interface to ROS ecosystem. It wraps some common ros methods. It is
 * the base class for most of the cnbiros_mobile library.
 *
 * RosInterface represents the base class to interface with ROS.  It wraps some
 * common ros methods and hides their complexity (i.e., subscribe and advertise
 * topics). It is directly derived derived from ros::NodeHandle, thus it shares
 * all its public and protected methods/members.
 * 
 * \par General description:
 * RosInterface can be used in two different way: on one hand can be
 * instanciated and used as a ros::NodeHandle class with the advantages of
 * several methods that hide some complexity of ros. On the other, it can be
 * derived in a new class and methods such as onRunning() can be implemented. In
 * this case, by calling the method Run(), the interface automatically spins
 * until the node is ros::ok(). In the loop, all the ros callbacks, services and
 * the derived method onRunning() are called at each iteration.
 * In this case, specific operations can be defined in the method onRunning() of
 * derived class.
 *
 * \par Basic methods and members:
 * - SetName() and GetName(): 
 * An important attribute of the class is its #name_.  This attribute is used to
 * automatically advertise and subscribe topics in its derived classes (and
 * possibly services)
 * - SetFrequency() and GetFrequency(): 
 * These methods are used to set and get the spinning node frequency.
 * - SetSubscriber() and GetSubscriber(): 
 * These methods are used to add a new subscriber to the class and to define the
 * related callback. By assumption a RosInterface might have many subscriber
 * - SetPublisher() and GetPublisher(): 
 * These methods are used to add/get a new publisher to the interface.
 * - Run(), Stop(), Resume(): 
 * These methods control the internal loop of the RosInterface()
 * - onRunning(), onStop(), onStart(): 
 * These callbacks can be implemented in the derived classes and they are
 * executed while the interface is running, once it stops or once it starts,
 * respectively
 *
 * \par Services:
 * By default, a RosInterface implements a standard ROS service to control the
 * state of the interface. By means of this service is possible to Start, Stop
 * and Resume the internal loop of the interface. The service is accessible via
 * ROS methods on <i>~/rosinterface_state</i>.
 *
 * \par Actions:
 * <i>NOT IMPLEMENTED YET</i>
 */

class RosInterface : public ros::NodeHandle {

	public:
		/*! \brief Constructor
		 *
		 * \param name 	Name of the interface
		 * \param ns 	Namespace of the node [default: "~"]
		 *
		 */
		RosInterface(std::string name, std::string ns="");

		//! \brief Destructor
		virtual ~RosInterface(void);

		/*! \brief Set interface name 
		 *
		 * The name of the interface will be used to automatically set most of
		 * the subscriber/publisher/services.
		 *
		 * \param 	name 	Name of the interface
		 */
		void SetName(const std::string name);

		/*! \brief Get interface name
		 * \return 			The interface name
		 */
		std::string GetName(void);

		/*! \brief Set interface frequency 
		 *
		 * The frequency of the interface represents the rate at which the loop
		 * is cycling.
		 *
		 * By default the frequency rate of the interface is set at
		 * #CNBIROS_NODE_FREQUENCY. The frequency can be changed also if the
		 * interface is already running.
		 *
		 * \param	frequency	Frequency of the interface
		 */
		void SetFrequency(float frequency);
		
		/*! \brief Get current interface frequency
		 * \return The interface frequency
		 */
		float GetFrequency(void);

		/*! \brief Get the interface subscriber on topic
		 *
		 * Returns a pointer of type ros::Subscriber related to required
		 * subscriber.
		 *
		 * \param	topic 	subscriber topic
		 * \return 		Pointer to the subscriber
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
		void SetSubscriber(const std::string topic, void(*fp)(M));
		
		/*! \brief Set a new interface subscriber on topic
		 * 	
		 * Add a new subscriber to the interface subscriber list. This method
		 * is used with a class method as callback. The message buffer size is
		 * set by default equal to #CNBIROS_MESSAGES_BUFFER.
		 * 
		 * \param topic 	Name of the subscribed topic
		 * \param fp 		Callback function to be called when a message
		 * 					arrives on the subscribed topic
		 * \param obj		Pointer to the object where the callback is
		 *					istantiated
		 */
		template<class M, class T>
		void SetSubscriber(const std::string topic, void(T::*fp)(M), T* obj);

		template<class M>
		void SetSubscriber(const std::string topic, const boost::function< void(const boost::shared_ptr<M const>&)> &callback);

		ros::Publisher* GetPublisher(std::string topic);

		/*! \brief Set a new interface publisher
		 *
		 * Create a new publisher that advertises on the required topic.  By
		 * assumption, a RosInterface has only a publisher. The message buffer
		 * size is set by default equal to #CNBIROS_MESSAGES_BUFFER.
		 * 
		 * \param topic 	Name of the advertise topic
		 */
		template<class M>
		void SetPublisher(std::string topic);

		/*! \brief Publish message on advertise topic. 
		 *
		 * The message buffer size is set by default equal to
		 * #CNBIROS_MESSAGES_BUFFER.
		 *
		 * \param msg 	Message to be published
		 */
		template<class M>
		void Publish(std::string topic, M& msg);

		/*! \brief Main run method
		 * 	
		 * Method to start the interface
		 */ 
		void Run(void);

		/*! \brief Stop the interface 
		 *
		 * \sa Resume(), IsStopped()
		 */ 
		void Stop(void);

		/*! \brief Restart the interface 
		 *
		 * \sa Stop(), IsStopped()
		 */
		void Resume(void);

		/*! \brief Check if the interface is stopped
		 *
		 * \return 			Stop flag
		 * 
		 * \sa Stop(), Resume()
		 */
		bool IsStopped(void);

		/*! \brief Set the frameid for the interface
		 */
		void SetFrame(const std::string frame);
		
		/*! \brief Get the frameid of the interface
		 *
		 * \return 		frame id of the interface
		 */
		std::string GetFrame(void);

		protected:
		/*! Callback to be executed while the interface is running
		 * 
		 * This callback is called at every iteration of the interface. Can be
		 * instanciated and defined in the derived class.
		 *
		 * \sa Run()
		 */
		virtual void onRunning(void) {};

		/*! Callback to be executed when the interface stops
		 * 
		 * This callback is called when the interface stops. Can be instanciated
		 * and defined in the derived class.
		 *
		 * \sa Stop()
		 */
		virtual void onStop(void) {};
		
		/*! Callback to be executed when the interface starts
		 * 
		 * This callback is called when the interface starts. Can be
		 * instanciated and defined in the derived class.
		 *
		 * \sa Start()
		 */
		virtual void onStart(void) {};

	private:
		
		bool on_rosinterface_service_(cnbiros_services::RosInterfaceState::Request &req,
									  cnbiros_services::RosInterfaceState::Response &res);

	protected:
		std::map<std::string, ros::Subscriber> 	rossubs_;
	private:
		//! Generic interface members
		std::string 	rosname_;
		float 			rosfrequency_;
		bool 			is_stopped_;
		
		ros::Rate* 								rosrate_; 						
		std::map<std::string, ros::Publisher> 	rospubs_;

		//! Services related members
		ros::ServiceServer rossrv_state_;

		//! Frame related members
		std::string rosframe_;
};

template<class M>
void RosInterface::SetSubscriber(const std::string topic, void(*fp)(M)) {
	this->rossubs_[topic] = this->subscribe<M>(topic, CNBIROS_MESSAGES_BUFFER, fp);
}

template<class M, class T>
void RosInterface::SetSubscriber(const std::string topic, void(T::*fp)(M), T* obj) {
	this->rossubs_[topic] = this->subscribe<M>(topic, CNBIROS_MESSAGES_BUFFER, fp, obj);
}

template<class M>
void RosInterface::SetSubscriber(const std::string topic, const boost::function< void(const boost::shared_ptr<M const>&)> &callback) {
	this->rossubs_[topic] = this->subscribe<M>(topic, CNBIROS_MESSAGES_BUFFER, callback);
}

template<class M>
void RosInterface::SetPublisher(std::string topic) {
	this->rospubs_[topic] = this->advertise<M>(topic, CNBIROS_MESSAGES_BUFFER);
}

template<class M>
void RosInterface::Publish(std::string topic, M& msg) {
	ros::Publisher* ptr_pub;
	ptr_pub = this->GetPublisher(topic);

	ptr_pub->publish(msg);
}


	}
}


#endif

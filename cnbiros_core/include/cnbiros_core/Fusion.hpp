#ifndef CNBIROS_CORE_FUSION_HPP
#define CNBIROS_CORE_FUSION_HPP

#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_core/SensorGrid.hpp"
#include "cnbiros_services/Reset.h"

#include "cnbiros_bci/TiCMessage.h"
#include "cnbiros_bci/TiDMessage.h"

namespace cnbiros {
	namespace core {

//* Fusion class
/**
 * \brief Fusion class is the base class to handle different kind of message and
 * eventually fuse them together
 *
 * Fusion() represents the base class to handle different kind of message (e.g.,
 * sensor messages, command messages), to convert them in a SensorGrid type and
 * eventually to fuse together with a specific algorithm and publish them on a
 * given topic.
 *
 * \par General description:
 * As derived class of RosInterface(), Fusion() can be used in two behaviours:
 * on one hand by implementing in a derived class the Process() function and
 * then, start the main loop (i.e., Run()). On the other, by using the method of
 * the class sequentially and implement a specific loop. Generally, the class
 * listen on different topics (added by the method AddSource()). Once a message
 * arrives in one of the topic, it converts it in a SensorGrid object (derived
 * by grid_map), by adding a new layer to the #grid_ member of the class. The
 * new layer has the same name of the topic from where the message was received.
 * If the interface is running (Run() method), first it process the time
 * percistency of the base layer (layer created automatically when the class is
 * instaciated with the name equals to the name of the class), second it calls
 * the method Process() to implement the fusion logic. Finally, it publishes the
 * SensorGrid message to the topic ("/" + GetName()).
 * 
 * 
 * \par Basic methods and members: 
 * - AddSource():
 * Methods to add a topic on the subscribers list of the class. In addition to
 * the name of the topic, the type of the expected message is required.
 * - SetDecayTime():
 * Methods to set the time persistency of the target layer. By default is set to
 * 0
 * 
 * - Services:
 * By default,  Fusion() class implements a ROS service to reset its own
 * SensorGrid. The service is accessible via ROS methods on <i>~/reset_ + GetName()</i>
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 *
 */

class Fusion : public RosInterface {
	
	public:
		/*! \brief Constructor
		 * \param	Name of the interface. It advertises on the topic
		 * 			/fusion_\p name. It creates a layer in its own grid with
		 * 			the povided \p name.
		 */
		Fusion(std::string name);

		/*! \brief Destructor
		 */
		virtual ~Fusion(void);

		/*! \brief Add a source for the class
		 *
		 * This methods subscribe on the given \p topic by define the \p type
		 * of message (Fusion::AsLaserScan, Fusion::AsPointCloud).
		 *
		 * \param 	topic 	The topic to be subscribed to
		 * \param 	type	The type of message
		 */
		void AddSource(std::string topic, const unsigned int type);

		/*! \brief Set the time persistency of the target layer
		 *
		 * \param 	time 	Time persistency [in seconds]. By default it is
		 * 					equal to 0
		 */
		void SetDecayTime(float time);

		/*! \brief Empty function to implement the fusion logic
		 *
		 * This method is called at every iteration if the interface is running.
		 *
		 */
		virtual void Process(void){};

	protected:
		virtual void onRunning(void);
		virtual void onStop(void);
		virtual void onStart(void);

	private:

		void onLaserScan(const sensor_msgs::LaserScan::ConstPtr& data, std::string topic);
		void onPointCloud(const sensor_msgs::PointCloud::ConstPtr& data, std::string topic);
		
		void process_decay(SensorGrid& grid, std::string target, float decayrate);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);

	public:
		static const unsigned int AsLaserScan 	= 1;
		static const unsigned int AsPointCloud 	= 2;

	protected:
		std::string 		topic_;
		SensorGrid 			grid_;

	private:
		ros::ServiceServer	srv_reset_;
		float 				decayrate_;
};

	}
}

#endif

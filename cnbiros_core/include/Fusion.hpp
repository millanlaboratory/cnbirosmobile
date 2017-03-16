#ifndef CNBIROS_CORE_FUSION_HPP
#define CNBIROS_CORE_FUSION_HPP

#include <string>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "RosInterface.hpp"
#include "SensorGrid.hpp"
#include "cnbiros_services/Reset.h"

namespace cnbiros {
	namespace core {

//* Fusion class
/**
 * Basic implementation of the class. This class derives directly from
 * RosInterface() base class. This class is in charged of fuse data from
 * different sensors (Sensor() class) with the assumption that each of them
 * provide a grid map. It publishes a new grid map on the topic <i>~/fusion</i>
 * with all the information of the sensor plus a new layer named fusion.
 * Different fusion logic can be implemented in derived classes.
 * 
 * <b>Behaviour:</b> \n
 * The class might subscribe on several topics (with grid_map_msgs::GridMap
 * type). Every time one of the subscribed topic has new data, the Fusion class
 * update its own grid on the source corresponding layer. While running, the
 * Fusion class fuses the different layers according to the following logic: it
 * sums all the existent layers (except the <i>fusion</i> layer) and it
 * thresholds the data on the <i>fusion</i> layer between -1 and 1. In
 * addition, it's possible to set a decay time for the <i>fusion</i> layer.
 * Fused data is published on topic <i>/fusion</i>. If a different publishing
 * topic is required, please consider ros remap tool instead of changing
 * manually the topic name.
 * 
 * 
 * <b>Basic methods and members:</b> \n 
 * <ul>
 * <li> Fusion(ros::NodeHandle* node, std::string name): 
 * Class constructor needs a pointer to the ros node handler and name of the
 * fusion. This name is set automatically during instanciation.  
 * <li> onStop(), onStart(): 
 * once the fusion starts or stops, its own grid map is reset to 0.  
 * <li> onRunning(): 
 * While running, the class fuse the current sensors' layers and it process the
 * time decay (if it is set). In addition, it publishes the resulting grid map
 * on the topic <i>/fusion</i>
 * <li>AddSource():
 * Methods to add a topic on the subscribers list of the class. It is assumed
 * the type of the message received is grid_map_msgs::GridMap.
 * <li>SetDecayTime():
 * Methods to set the decay time.
 * </ul>
 * 
 * <b>Services:</b> \n 
 * By default,  Fusion() class implements a ROS service to reset its own grid
 * map.  The service is accessible via ROS methods on <i>~/gridmap_reset</i>
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 *
 */

class Fusion : public RosInterface {
	
	public:
		/*! \brief Constructor
		 *
		 * Constructor with pointer to ROS node handler and the name of the
		 * fusion. By default the name is set to #CNBIROS_FUSION_NAME
		 *
		 * \param node 	Pointer to the ROS node handler 
		 * \param name 	Name of the sensor
		 */
		Fusion(std::string name = CNBIROS_FUSION_NAME);
		
		//! \brief Destructor
		virtual ~Fusion(void);

		/*! \brief Add a subscriber on the given topic
		 *
		 * Method to add a subscriber to the subscribers list on the given
		 * topic name. The Fusion class except that subscribed topics provide
		 * grid_map_msgs::GridMap messages
		 *
		 * \param topic 	Name of the topic
		 */
		void AddSource(std::string topic);
		
		/*! \brief Set the decay time for the fusion grid map 
		 *
		 * Method to set the decay time of the grid map of the class. It
		 * represents the time from a point in the grid to decreases to 0 (if
		 * it is not fired again). If the running rate of the node is changes,
		 * also the decay time is updated automatically. At the instantiation,
		 * the decay time is set to 0.
		 *
		 * \param time 	Time of decay in seconds
		 */
		void SetDecayTime(float time);

	protected:
		/*! Callback to be executed while the sensor is running
		 * 
		 * This callback is called at every iteration of the sensor. It
		 * processes the time decay (if it is set) and fuse the all existing
		 * layers.
		 *
		 * \sa RosInterface::Run()
		 */
		virtual void onRunning(void);
		
		/*! Callback to be executed when the object stops
		 * 
		 * This callback is called when the object stops. By default, it resets
		 * the #rosgrid_ of the object.  
		 *
		 * \sa RosInterface::Stop()
		 */
		virtual void onStop(void);
		
		/*! Callback to be executed when the object starts
		 * 
		 * This callback is called when the object starts. By default, it resets
		 * the #rosgrid_ of the object.  
		 *
		 * \sa RosInterface::Stop()
		 */
		virtual void onStart(void);
	
		/*! Method implementing the fusion logic
		 * 
		 * This method implements the fusion logic on a given target layer.
		 * This implementation has the following logic: summing all existent
		 * layers (but target) and thresholding between -1 and 1.
		 * 
		 * \param map 		The grid map object to be fused
		 * \param target	The name of the target layer on the grid map object
		 */
		virtual void process_fusion(SensorGrid& grid, std::string target);
		
		/*! Method to process the decay
		 * 
		 * This method process the time decay following a linear decay rate.
		 *
		 * \param map 			The grid map object to be processed
		 * \param target		The name of the target layer on the grid map object
		 * \param decayrate	The decay rate
		 */
		virtual void process_decay(SensorGrid& grid, std::string target, float decayrate);	

	private:
		void rosfusion_callback_(const grid_map_msgs::GridMap& msg);
		bool on_service_reset_(cnbiros_services::Reset::Request& req,
							   cnbiros_services::Reset::Response& res);

	protected:
		float 				decayrate_;
		std::string 		rostopic_pub_;
		SensorGrid 			rosgrid_;

	private:
		ros::ServiceServer	rossrv_reset_;
};

	}
}

#endif

#ifndef CNBIROS_CORE_SENSOR_HPP 
#define CNBIROS_CORE_SENSOR_HPP

#include <ros/ros.h> 
#include <grid_map_ros/grid_map_ros.hpp> 
#include <grid_map_msgs/GridMap.h>

#include "Flags.hpp" 
#include "RosInterface.hpp" 
#include "GridMapTool.hpp"
#include "cnbiros_services/GridMapReset.h"

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

class Sensor : public RosInterface {

	public:
		/*! \brief Constructor
		 *
		 * Constructor with pointer to ROS node handler and the name of the
		 * sensor
		 *
		 * \param node 	Pointer to the ROS node handler 
		 * \param name 	Name of the sensor
		 */
		Sensor(ros::NodeHandle* node, std::string name);
		
		//! \brief Destructor
		virtual ~Sensor(void);

	protected:
		/*! Callback to be executed when the sensor stops
		 * 
		 * This callback is called when the sensor stops. By default, it resets
		 * the #rosgrid_ of the sensor.  
		 *
		 * \sa RosInterface::Stop()
		 */
		virtual void onStop(void);
		
		/*! Callback to be executed when the sensor starts
		 * 
		 * This callback is called when the sensor starts. By default, it resets
		 * the #rosgrid_ of the sensor.  
		 *
		 * \sa RosInterface::Start()
		 */
		virtual void onStart(void);
		
		/*! Callback to be executed while the sensor is running
		 * 
		 * This callback is called at every iteration of the sensor. Can be
		 * instanciated and defined in the derived class.  
		 *
		 * \sa RosInterface::Run()
		 */
		virtual void onRunning(void) = 0;

	private: 
		bool on_gridmap_reset_(cnbiros_services::GridMapReset::Request& req,
				cnbiros_services::GridMapReset::Response& res);

	protected: 
		grid_map::GridMap 		rosgrid_; 
		std::string				sensor_layer_; 
		std::string 			rostopic_grid_;

	private: 
		ros::ServiceServer 		rossrv_reset_;
		
};


	} 
}

#endif

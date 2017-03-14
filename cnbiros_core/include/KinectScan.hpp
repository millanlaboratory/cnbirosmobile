#ifndef CNBIROS_CORE_KINECTSCAN_HPP
#define CNBIROS_CORE_KINECTSCAN_HPP

#include <sensor_msgs/LaserScan.h>

#include "Flags.hpp"
#include "Sensor.hpp"

namespace cnbiros {
	namespace core {

//* KinectScan class
/**
 * Implementation of the KinectScan sensor. This class derives directly from
 * Sensor() base class. It reprensents the implementation of a kinectscan
 * object. It is based on data provided by the pointcloud_to_laserscan standard
 * ros package. In brief, this package project the point cloud read by the
 * kinect on the plane simulating a laser scan.
 * 
 * <b>Behaviour:</b> \n
 * While running, KinectScan class acquires data from the default
 * pointcloud_to_laserscan topic (<i>/camera/scan/</i> defined in
 * #CNBIROS_KINECTSCAN_TOPIC), it converts the data in a grid map and it
 * publishes the grid_map_msgs on the <i>sensor_kinectscan</i> topic. Class
 * name is set by default to "kinectscan" (defined in
 * #CNBIROS_KINECTSCAN_NAME). If a different publishing/subscribing topic is
 * required, please consider ros remap tool instead of changing manually the
 * topic name.
 * 
 * <b>Basic methods and members:</b> \n 
 * Same methods and members as Sensor() and RosInterface().
 * 
 * <b>Services:</b> \n 
 * Same services as Sensor() and RosInterface().
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 */
class KinectScan : public Sensor {

	public:

		/*! \brief Constructor
		 *
		 * Constructor with pointer to ROS node handler and the name of the
		 * sensor
		 * 
		 * By default the name of the class is set to #CNBIROS_KINECTSCAN_NAME
		 *
		 * \param node 	Pointer to the ROS node handler 
		 * \param name 	Name of the sensor
		 */
		KinectScan(std::string name = CNBIROS_KINECTSCAN_NAME);
		
		//! \brief Destructor
		~KinectScan(void);

		/*! \brief Implementation of the callback during running
		 *
		 * It implements the conversion between the sensor_msgs::LaserScan
		 * message (provided by pointcloud_to_laserscan package) and the grid
		 * map. The callback is called at every iteration.
		 */ 
		void onRunning(void);

	private:
		void roskinect_callback_(const sensor_msgs::LaserScan& msg);

};

	}
}


#endif

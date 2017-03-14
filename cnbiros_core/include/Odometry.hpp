#ifndef CNBIROS_CORE_ODOMETRY_HPP
#define CNBIROS_CORE_ODOMETRY_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Flags.hpp"
#include "RosInterface.hpp"
#include "cnbiros_services/OdometryReset.h"

namespace cnbiros {
	namespace core {

//* Odometry class
/**
 * Odometry() class represents the base class for any kind of odometry developed
 * in the cnbiros_mobile framework. It is a pure abstract class directly derived
 * from RosInterface(). It provides attributes and it implements methods common
 * to different types of odometries. In particular, it has a nav_msgs::Odometry
 * (#rosodom_msg_) and methods to set/reset such a message. 
 * 
 * <b>Behaviour</b> \n
 * The behaviour of the class depends on the implementation of the onRunning()
 * callback.
 * 
 * <b>Basic methods and members:</b> \n 
 * Same methods and members as RosInterface(). In addition:
 * <ul>
 * <li> onReset(): virtual method to be implemented in the derived classes to
 * reset the odometry of the device.
 * </ul>
 * 
 * <b>Services:</b> \n 
 * Same services as RosInterface(). In addition, it implements a ROS service to
 * reset its own odometry message. The service is accesible via ROS methods on
 * <i>~/odometry_reset</i>
 * 
 * <b>Actions:</b> \n 
 * <i>NOT IMPLEMENTED YET</i>
 *
 */

class Odometry : public RosInterface {

	public:
		/*! \brief Constructor
		 *
		 * Constructor with pointer to ROS node handler and the name of the
		 * odometry
		 *
		 * \param node 	Pointer to the ROS node handler 
		 * \param name 	Name of the odometry
		 */
		Odometry(std::string name);
		
		//! \brief Destructor
		virtual ~Odometry(void);

	protected:
		
		/*! Callback to be executed while the odometry is running
		 * 
		 * This callback is called at every iteration of the odometry. Can be
		 * instanciated and defined in the derived class.  
		 *
		 * \sa RosInterface::Run()
		 */
		virtual void onRunning(void) = 0;
		
		/*! Callback to be executed when the reset service is required 
		 * 
		 * This callback is hardware specific. Thus, it must be implemented in
		 * the derived classes
		 *
		 */
		virtual void onReset(void) = 0;

		/*! Reset to 0 the odometry message
		 * 
		 */
		void reset_message(void);
		
		/*! Set the odometry message
		 *
		 * Given the required x, y, z, omega (orientation) positions and vx, vy,
		 * vz, vomega (angular) velocities and the current sequence number, this
		 * method fill the odometry message accordingly.
		 * 
		 */
		void set_message(float x, float y, float z, float omega,
						 float vx, float vy, float vz, float vomega, 
						 unsigned int sequence);

	private:
		 virtual bool on_odometry_reset_(cnbiros_services::OdometryReset::Request& req,
										 cnbiros_services::OdometryReset::Response& res);

	protected:
		nav_msgs::Odometry 	rosodom_msg_;
		ros::ServiceServer	rossrv_reset_;
		std::string 		rostopic_pub_;
};


	}
}


#endif

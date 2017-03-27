#ifndef CNBIROS_CORE_ODOMETRY_HPP
#define CNBIROS_CORE_ODOMETRY_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_services/SetOdometry.h"
#include "cnbiros_services/Reset.h"

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

		void GetOdometry(nav_msgs::Odometry& odom);
		virtual void SetOdometry(const nav_msgs::Odometry& odom) = 0;
		void Reset(void);

	protected:
		
		/*! Callback to be executed while the odometry is running
		 * 
		 * This callback is called at every iteration of the odometry. Can be
		 * instanciated and defined in the derived class.  
		 *
		 * \sa RosInterface::Run()
		 */
		virtual void onRunning(void) {};
	
		virtual void onStop(void);
		virtual void onStart(void);
		
	private:
		virtual bool on_service_set_(cnbiros_services::SetOdometry::Request& req,
								     cnbiros_services::SetOdometry::Response& res);
		virtual bool on_service_reset_(cnbiros_services::Reset::Request& req,
										cnbiros_services::Reset::Response& res);
		
	protected:
		std::string 		topic_;
		nav_msgs::Odometry 	odometry_data_;
	
	private:
		ros::ServiceServer	srv_set_;
		ros::ServiceServer	srv_reset_;
};


	}
}


#endif

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
 * \brief Odometry class is the base class to interact with robot odometry.
 *
 * Odometry() class represents the base class for any kind of odometry developed
 * in the cnbiros_mobile framework.
 *
 * \par General description:
 * The basic behavior of the Odometry() class is to read the odometry from the
 * robot encoder and publish it in the given topic ("/odom"). If a different
 * publishing topic is required, please consider ros remap tool instead of
 * changing manually the topic name.
 * 
 * \par Basic methods and members:
 * - GetOdometry(), SetOdometry(): 
 * These two methods are used to get and set the odometry od the robot,
 * respectively. In particular, SetOdometry() is a pure abstract method specific
 * for each hardware and to be implemented in a derived class.
 * - Reset():
 * This methos resets the current odometry of the robot.
 * - onStop(), onStart():
 * Callbacks called when the class is required to stop/start.
 * - onRunning():
 * Main callback called when the class is running.
 
 * \par Services:
 * By default, the Odometry() class implements two service to set a given odometry
 * or to reset (to 0) the current one. The services are accessible via ROS
 * methods on <i>set_odometry</i> and <i>reset_odometry</i>, respectively.
 *
 */

class Odometry : public RosInterface {

	public:
		/*! \brief Constructor
		 *
		 * \param name 	Name of the class
		 */
		Odometry(std::string name);
		
		//! \brief Destructor
		virtual ~Odometry(void);

		/*! \brief Get the current odometry
		 *
		 * \param[out] odom 	Reference to an odometry message
		 */
		void GetOdometry(nav_msgs::Odometry& odom);

		/*! \brief Pure virtual function to set the odometry
		 * 
		 * \param odom 	Reference to the odometry to be set
		 */
		virtual void SetOdometry(const nav_msgs::Odometry& odom) = 0;
		
		/*! \brief Reset the current odometry to 0
		 */
		void Reset(void);

	protected:
		
		/*! \brief Empty virtual callback executed when the interface is running
		 */
		virtual void onRunning(void) {};
	
		/*! \brief Callback when the interface stops
		 * 
		 * By default, the odometry is set to 0.
		 */
		virtual void onStop(void);
		
		/*! \brief Callback when the interface starts
		 * 
		 * By default, the odometry is set to 0.
		 */
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

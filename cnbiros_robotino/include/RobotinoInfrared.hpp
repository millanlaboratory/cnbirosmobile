#ifndef ROBOTINOINFRARED_HPP
#define ROBOTINOINFRARED_HPP

#include <algorithm>
#include <rec/robotino/api2/DistanceSensorArray.h>

#include "Robotino.hpp"
#include "RobotinoCom.hpp"
#include "Sensor.hpp"

#define ROBOTINO_BASE_RADIUS 			0.2f		// Radius of the base 				[meters]
#define ROBOTINO_INFRARED_MAX_DISTANCE	0.6f 		// Max distance read by the sensors [meters]
#define ROBOTINO_INFRARED_ANGLE		 	(40.0f * M_PI)/180.0f
													// Angle (in radians) between two sensors
#define ROBOTINO_INFRARED_GRID_XDIM 	2.0f 		// x-dimension of the grid 			[meters]
#define ROBOTINO_INFRARED_GRID_YDIM 	2.0f 		// y-dimension of the grid 			[meters]
#define ROBOTINO_INFRARED_GRID_RES 		0.02f 		// resolution of the grid 			[meters]
#define ROBOTINO_INFRARED_GRID_LAYER  	"raw"
#define ROBOTINO_INFRARED_GRID_FRAMEID	"robot"

namespace cnbiros {
	namespace robotino {



class RobotinoInfrared : public core::Sensor, public  rec::robotino::api2::DistanceSensorArray {

	public:
		RobotinoInfrared(std::string 	hostname, 
						 unsigned int 	frequency,
						 float 			xdim    = ROBOTINO_INFRARED_GRID_XDIM,
						 float 		    ydim    = ROBOTINO_INFRARED_GRID_YDIM,
						 float          res 	= ROBOTINO_INFRARED_GRID_RES,
						 std::string 	layer   = ROBOTINO_INFRARED_GRID_LAYER,
						 std::string 	frameid = ROBOTINO_INFRARED_GRID_FRAMEID);
		virtual ~RobotinoInfrared(void) {};
	
		void SetDecayTime(float time);
		void distancesChangedEvent(const float* distances, unsigned int size);

		void Process(void);
	protected:
		void ProcessDecay(void);
		void gaussian(void);

	protected:
		RobotinoCom* com_;

	private:
		grid_map::GridMap grid_;
		grid_map_msgs::GridMap msg_;
		float decaystep_;
		

};


	}
}




#endif

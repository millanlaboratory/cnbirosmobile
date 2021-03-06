#ifndef FLAGS_HPP
#define FLAGS_HPP

// Generic default definitions
#define CNBIROS_NODE_FREQUENCY 			10.0f 				// Default update frequency of nodes [Hz]
#define CNBIROS_MESSAGES_BUFFER 		100					// Default size of message buffer

// GridMap default definitions
#define CNBIROS_SENSORGRID_X 		5.0f
#define CNBIROS_SENSORGRID_Y 		5.0f
#define CNBIROS_SENSORGRID_R 		0.05f

// Fusion default definitions
#define CNBIROS_FUSION_NAME 		"fusion"

// ForceField default definitions
#define CNBIROS_FORCEFIELD_NAME 			"forcefield"
#define CNBIROS_FORCEFIELD_GRIDLAYER 		"fusion"
#define CNBIROS_FORCEFIELD_OBSTRUCTION 		0.4f		// Default obstruction (size of the device) [meters] 
#define CNBIROS_FORCEFIELD_SPATIALDECAY 	0.5f		// Default spatial decay of attractors/repellors

// Odometry default definitions
#define CNBIROS_ODOMETRY_NAME 		"odom"

// RobotBase default definitions
#define CNBIROS_MOTORS_NAME 	"motors"
#define CNBIROS_MOTORS_TOPIC 	"/cmd_vel"



#endif

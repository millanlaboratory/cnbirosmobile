#ifndef FLAGS_HPP
#define FLAGS_HPP

#define CNBIROS_NODE_FREQUENCY 			10.0f 				// Default update frequency of nodes [Hz]
#define CNBIROS_MESSAGES_BUFFER 		100					// Default size of message buffer
#define CNBIROS_TOPIC_ODOMETRY 			"/odom"				// Default odometry topic according to standards 

#define CNBIROS_SENSOR_GRID_X			5.0f				// Default grid map size for sensor [m]
#define CNBIROS_SENSOR_GRID_Y			5.0f				// Default grid map size for sensor [m]
#define CNBIROS_SENSOR_GRID_R			0.1f				// Default grid map resolution for sensor [m]

#define CNBIROS_ROBOT_NODE_FREQUENCY 	10.0f				// Frequency of node main loop [Hz]
#define CNBIROS_SENSOR_NODE_FREQUENCY	10.0f				// Frequency of node main loop [Hz]
#define CNBIROS_CTRL_NODE_FREQUENCY		10.0f				// Frequency of node main loop [Hz]

#define CNBIROS_TOPIC_FUSION 			"/sensors_fusion" 	// Default sensor fusion topic
#define CNBIROS_TOPIC_VELOCITY 			"/base_velocity" 	// Default motor velocity topic
#define CNBIROS_TOPIC_KINECTSCAN 		"/camera/scan"



// NEW FLAGS (for sure used)

// GridMap default definitions
#define CNBIROS_GRIDMAP_XSIZE 		5.0f
#define CNBIROS_GRIDMAP_YSIZE 		5.0f
#define CNBIROS_GRIDMAP_RESOLUTION 	0.05f
#define CNBIROS_GRIDMAP_FRAME 		"base_link"

// Kinectscan default definitions
#define CNBIROS_KINECTSCAN_NAME 	"kinectscan"
#define CNBIROS_KINECTSCAN_TOPIC 	"/camera/scan"

// Fusion default definitions
#define CNBIROS_FUSION_NAME 		"fusion"

#endif

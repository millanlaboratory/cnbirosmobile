#ifndef FLAGS_HPP
#define FLAGS_HPP

#define CNBIROS_MESSAGES_BUFFER 		1000				// Size of message buffer
#define CNBIROS_ROBOT_NODE_FREQUENCY 	10.0f				// Frequency of node main loop [Hz]
#define CNBIROS_SENSOR_NODE_FREQUENCY	10.0f				// Frequency of node main loop [Hz]
#define CNBIROS_CTRL_NODE_FREQUENCY		10.0f				// Frequency of node main loop [Hz]

#define CNBIROS_TOPIC_FUSION 			"/sensors_fusion" 	// Default sensor fusion topic
#define CNBIROS_TOPIC_VELOCITY 			"/motor_velocity" 	// Default motor velocity topic
#define CNBIROS_TOPIC_ODOMETRY			"/base_odometry" 	// Default odometry topic

#endif

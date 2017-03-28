#ifndef CNBIROS_ROBOTINO_COMMUNICATION_HPP
#define CNBIROS_ROBOTINO_COMMUNICATION_HPP

#include <rec/robotino/api2/Com.h>
#include <ros/ros.h>

namespace cnbiros {
	namespace robotino {

//* Robotino Communication class
/**
 * \brief Class to manage communication to robotino base
 *
 * This class handles the communication from/to robotino base. It implements the
 * rec::robotino::api2 communication callbacks (e.g., on error, on connection
 * and on disconnection) and it provides method for connection and
 * disconnection.
 * 
 * It is used in each robotino class of this library.
 */
class Communication : public rec::robotino::api2::Com {

	public:
		/*! \brief Constructor
		 *	
		 *	Constructor of the class with the name of the owner (for printing
		 *	out)
		 *
		 * \param owner	Name of the owner of the communication object
		 */ 
		Communication(std::string owner);

		/*! \brief Destructor
		 */
		~Communication(void);

		/*! \brief Connects to the robotino base
		 *
		 * This method connects to the robotino base identified by the given address.
		 * It tries to disconnect the current connection (if exists), it stores
		 * the address for further usage and it tries to connect to the robotino
		 * base.
		 *
		 * \param 	address 	IP address of the robotino base
		 */
		void Connect(std::string address);

		/*! \brief Disconnects from the robotino base
		 *
		 * This method checks if a connection exists and in the case, it
		 * disconnect it
		 */
		void Disconnect(void);

		/*! \brief Check if there is connection to the robotino base
		 *
		 * The method checks if there is a current active connection to the
		 * robotino base.
		 *
		 * \return 		True if connected, False othwerwise
		 */
		bool IsConnected(void);

		/*! \brief Get the address of the connection
		 *
		 * The method returns the address of the last connection performed by
		 * the method Connect().
		 *
		 * \return 	The address of the robotino base
		 */
		std::string GetAddress(void);

		/*! \brief Set the address of the connection
		 *
		 * The method stores the address of the connection.
		 *
		 * \param address The IP address of the robotino base
		 *
		 * \remark This address is not used for the connection since Connect
		 * method requires to provide the address (aka -> useless).
		 */
		void SetAddress(std::string address);

		/*! \brief Displays occuring errors
		 */
		void errorEvent(const char* errorString);

		/*! \brief Displays connection event
		 */
		void connectedEvent(void);

		/*! \brief Display disconnection event
		 */
		void connectionClosedEvent(void);

	private:
		std::string owner_;
		std::string address_;
};

	}
}

#endif

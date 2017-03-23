#ifndef CNBIROS_BCI_CNBIINTERFACE_HPP
#define CNBIROS_BCI_CNBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

namespace cnbiros {
	namespace bci {

//* CnbiInterface class
/** \brief The class handles interactions to the CNBI Loop instance.
 *
 * \par General description:
 * CnbiInterface wraps some functionalities of ClLoop class (from cnbiloop
 * library) and provide some general methods to connect/disconnect to/from the
 * loop.
 *
 * \todo 
 * - Deciding if use this as base class for TiCProxy and TiDProxy.
 * - Add cnbilog handler
 */

class CnbiInterface {

	public: 
		/*! \brief Constructor
		 *
		 * Instanciate the class and configure the CNBI Loop
		 * to the given address.
		 *
		 * \param 	address 	Address of the loop [default: ""]
		 */
		CnbiInterface(const CcAddress address = "");

		//! \brief Destructor
		~CnbiInterface(void);

		/*! \brief Try to connect to CNBI Loop
		 *
		 * This method tries to connect to CNBI Loop. According to the argument
		 * \a the method is blocking.
		 *
		 * \param 	wait 	[default: true] If true the method is blocking,
		 * 					until the connection succed.
		 */
		bool Connect(bool wait = true);

		//! \brief Disconnect from the CNBI Loop
		void Disconnect(void);

		/*! \brief Get the address of the loop
		 *
		 * This method returns the address provided in the constructor.
		 *
		 * \return 		The address of the CNBI Loop provided in the constructor
		 */
		std::string GetAddress(void);

	private:
		CcAddress cnbiaddress_;



};

	}
}

#endif

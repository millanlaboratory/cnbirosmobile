#ifndef CNBIROS_BCI_TICPROXY_HPP
#define CNBIROS_BCI_TICPROXY_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiIc.hpp>
#include <tobiic/ICClassifier.hpp>
#include <cnbicore/CcProxy.hpp>

#include "RosInterface.hpp"
#include "TobiIcTools.hpp"
#include "cnbiros_bci/TiCMessage.h"

namespace cnbiros {
	namespace bci {

//* TiCProxy class
/** \brief The class handles multiple Tobi Interface C (TiC) connections to and
 * from the CNBI loop. For the generic ROS node that implements this class,
 * please refer to tobi_interface_c.cpp 
 *
 * \par General description:
 * TiCProxy and TiDProxy classes have similar behavior.  TiCProxy class
 * implements functionalities in order to handle in/out communications between
 * ROS and the CNBI loop. It is based on ClTobiIc interface (from cnbiloop
 * library) and on ICMessage, ICClassifier and ICClass from tobi classes (in
 * tobicore and tobiic libraries).  Once instanciated, it is possible to create
 * multiple ClTobiIc connections to the loop both as reader or as writer (i.e.,
 * "ClTobiIc::GetOnly" and "ClTobiIc::SetOnly", respectively). As in the case of
 * ClTobiIc is not possible to create a bi-directional connection on same pipe.
 *
 * \par Connection to CNBI loop:
 * The connection to the CNBI loop is achieved by the Attach() method and it is
 * defined by:
 * - the modality (e.g., TiCProxy::AsReader or TiCProxy::AsWriter) 
 * - the name of the pipe to be connected to (e.g., "/ctrl0", "/ctrl1")
 * - the topic to be subscribe on in case of TiCProxy::AsWriter modality
 * Notice that the modalities (TiCProxy::AsReader or TiCProxy::AsWriter) are
 * meant with respect to the cnbiloop pipe.
 *
 * \par Main behavior:
 * Once running, the TiCProxy implements two parallel behaviors:
 * - Checking for new ICMessage on the attached pipes and in case publishing the
 *   message to the default /ticproxy topic [TiCProxy::AsReader modality]
 * - Checking for new cnbiros_bci::TiCMessage on the subscribed topics and in
 *   case, sending them to the corresponding pipe [TiCProxy::AsWriter modality]
 * 
 * \par
 * A parallel implementation of ICMessage is provided as ROS message in this
 * library. The conversion between ICMessage and cnbiros_bci::TiCMessage is done
 * by the support class TobiIcTools.
 * 
 * \par Special behavior:
 * An automatic reconnection (aka re-Attach) functionalities is provided. On one
 * side, at each iteration, the class checks for the connection status of the
 * interfaces attached TiCProxy::AsReader and in case of detachment, it tries to
 * re-attach to the same pipe. On the other side, every time a ros message (aka
 * cnbiros_bci::TiCMessage) is received on the subscribed topics, it checks if
 * the related interface is attached and in case, it tries to re-attach to the
 * same pipe (in TiC::AsWriter modality).
 *
 * \par Usage:
 * The class can be used via the API provided by this library. However, a
 * generic implementation of a ROS node is already provided, and most of the
 * parameters (e.g., pipe names, topics) can be set via xml launch file. For
 * more details, please refer to tobi_interface_c.cpp executable.
 *
 * \sa TobiIcTools, tobi_interface_c.cpp
 *
 */

class TiCProxy : public core::RosInterface {
	
	public:
		/*! \brief Constructor
		 *
		 * The constructor of the class. As optional argument, the name of the
		 * interface. Notice that the eventual publishing will be executed on
		 * the topic "/" + $name.
		 *
		 * \param name Name of the interface [default: "ticproxy"]
		 *
		 */ 
		TiCProxy(std::string name = "ticproxy");

		//! \brief Destructor
		virtual ~TiCProxy(void);

		/*! \brief Attach to a given TobiInterface in the CNBI loop
		 *
		 * This method creates a new ClTobiIc object according to the \a mode
		 * provided and it tries to attach to the given \a pipe. In the case of
		 * TiCProxy::AsWriter, it also subscribe to the given ROS \a topic.
		 *
		 * \param 	mode	Modality of the connection (TiCProxy::AsReader,
		 * 					TiCProxy::AsWriter)
		 * \param 	pipe 	Identifier of the CNBI pipe to be used
		 * \param 	topic 	Mandatory argument in the case of TiCProxy::AsWriter
		 * 					\a mode. Identifier of the ROS topic to be used for
		 * 					subscription.
		 * \return 			Status of the connection
		 */
		bool Attach(unsigned int mode, std::string pipe, std::string topic = "");

		/*! \brief Detach the connection from a CNBI pipe
		 *
		 * This method detach an existing connection to the CNBI loop.
		 *
		 * \param 	pipe	Identifier of the connection (via pipe name)
		 * \return 			True if the connection is detached. False if it is
		 * 					could not be detached or a connection to the given 
		 * 					\a pipe does not exist
		 */
		bool Detach(std::string pipe);

		/*! \brief Check if the connection is attached
		 *
		 * This method checks if the connection identified by the \a pipe name
		 * is attached.
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection is attached. False if it is
		 * 					not attached or a connection to the given \a pipe
		 * 					does not exist
		 */
		bool IsAttached(std::string pipe);

		/*! \brief Check if the connection is set as writer
		 * 
		 * This method checks if the connection identified by the \a pipe name
		 * is set as writer (aka, ClTobiIc::SetOnly).
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection mode is TiCProxy::AsWriter. 
		 * 					False if it is not or a connection to the given \a 
		 * 					pipe does not exist
		 */
		bool IsWriter(std::string pipe);
		
		/*! \brief Check if the connection is set as reader
		 * 
		 * This method checks if the connection identified by the \a pipe name
		 * is set as reader (aka, ClTobiIc::GetOnly).
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection mode is AsReader. False if 
		 * 					it is not or a connection to the given \a pipe
		 * 					does not exist
		 */
		bool IsReader(std::string pipe);

	private:
		void onRunning(void);
		void onReceived(const cnbiros_bci::TiCMessage& msg);
	
	public:
		//! \brief AsReader modality for a connection (aka, ClTobiIc::GetOnly)
		const static unsigned int AsReader = 0;
		
		//! \brief AsWriter modality for a connection (aka, ClTobiIc::SetOnly)
		const static unsigned int AsWriter = 1;

	private:
		std::map<std::string, ClTobiIc*> 	ic_map_;
		std::map<std::string, std::string>	pt_map_;
		std::map<std::string, unsigned int> pm_map_;


};


	}
}


#endif

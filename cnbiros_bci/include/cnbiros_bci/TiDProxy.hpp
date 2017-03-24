#ifndef CNBIROS_BCI_TIDPROXY_HPP
#define CNBIROS_BCI_TIDPROXY_HPP

#include <ros/ros.h>
#include <cnbiloop/ClTobiId.hpp>

#include "cnbiros_core/RosInterface.hpp"
#include "cnbiros_bci/TobiIdTools.hpp"
#include "cnbiros_bci/TiDMessage.h"

namespace cnbiros {
	namespace bci {

//* TiDProxy class
/** \brief The class handles multiple Tobi Interface D (TiD) connections to and
 * from the CNBI loop. For the generic ROS node that implements this class,
 * please refer to tobi_interface_d.cpp 
 *
 * \par General description:
 * TiCProxy and TiDProxy classes have similar behavior.  TiDProxy class
 * implements functionalities in order to handle in/out communications between
 * ROS and the CNBI loop. It is based on ClTobiId interface (from cnbiloop
 * library) and on IDMessage from tobi classes (in tobicore and tobiid
 * libraries).  Once instanciated, it is possible to create multiple ClTobiId
 * connections to the loop as reader, as writer or as reader/writer (i.e.,
 * "ClTobiId::GetOnly", "ClTobiId::SetOnly" and "ClTobiId::SetGet",
 * respectively).
 *
 * \par Connection to CNBI loop:
 * The connection to the CNBI loop is achieved by the Attach() method and it is
 * defined by:
 * - the modality (e.g., TiDProxy::AsReader, TiDProxy::AsWriter or
 * 	 TiDProxy::AsReaderWriter) 
 * - the name of the pipe to be connected to (e.g., "/bus", "/dev")
 * - the topic to be subscribe on in case of TiDProxy::AsWriter or
 *   TiDProxy::AsReaderWriter modality
 * Notice that the modalities (TiDProxy::AsReader, TiDProxy::AsWriter and
 * TiDProxy::AsReaderWriter) are meant with respect to the cnbiloop pipe.
 *
 * \par Main behavior:
 * Once running, the TiDProxy implements two parallel behaviors:
 * - Checking for new IDMessage on the attached pipes and in case publishing the
 *   message to the default /tidproxy topic [TiDProxy::AsReader or
 *   TiDProxy::AsReaderWriter modalities]
 * - Checking for new cnbiros_bci::TiDMessage on the subscribed topics and in
 *   case, sending them to the corresponding pipe [TiDProxy::AsWriter or
 *   TiDProxy::AsReaderWriter modalities]
 * 
 * \par
 * A parallel implementation of IDMessage is provided as ROS message in this
 * library. The conversion between IDMessage and cnbiros_bci::TiDMessage is done
 * by the support class TobiIdTools.
 * 
 * \par Special behavior:
 * An automatic reconnection (aka re-Attach) functionalities is provided. On one
 * side, at each iteration, the class checks for the connection status of the
 * interfaces attached TiDProxy::AsReader (or TiDProxy::AsReaderWriter) and in
 * case of detachment, it tries to re-attach to the same pipe. On the other
 * side, every time a ros message (aka cnbiros_bci::TiDMessage) is received on
 * the subscribed topics, it checks if the related interface is attached and in
 * case, it tries to re-attach to the same pipe (according to the modality set
 * when the connection was created).
 *
 * \par Usage:
 * The class can be used via the API provided by this library. However, a
 * generic implementation of a ROS node is already provided, and most of the
 * parameters (e.g., pipe names, topics) can be set via xml launch file. For
 * more details, please refer to tobi_interface_d.cpp executable.
 *
 * \sa TobiIdTools, tobi_interface_d.cpp
 *
 */

class TiDProxy : public core::RosInterface {
	
	public:
		/*! \brief Constructor
		 *
		 * The constructor of the class. As optional argument, the name of the
		 * interface. Notice that the eventual publishing will be executed on
		 * the topic "/" + $name.
		 *
		 * \param name Name of the interface [default: "tidproxy"]
		 *
		 */ 
		TiDProxy(std::string name = "tidproxy");
		
		//! \brief Destructor
		virtual ~TiDProxy(void);
	
		/*! \brief Attach to a given TobiInterface in the CNBI loop
		 *
		 * This method creates a new ClTobiId object according to the \a mode
		 * provided and it tries to attach to the given \a pipe. In the case of
		 * TiDProxy::AsWriter or TiDProxy::AsReaderWriter, it also subscribe to
		 * the given ROS \a topic.
		 *
		 * \param 	mode	Modality of the connection (TiDProxy::AsReader,
		 * 					TiDProxy::AsWriter, TiDProxy::AsReaderWriter)
		 * \param 	pipe 	Identifier of the CNBI pipe to be used
		 * \param 	topic 	Mandatory argument in the case of TiDProxy::AsWriter
		 * 					or TiDProxy::AsReaderWriter \a mode. Identifier of 
		 * 					the ROS topic to be used for subscription.
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
		 * is set as writer (aka, ClTobiId::SetOnly or ClTobiId::SetGet).
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection mode is TiDProxy::AsWriter or
		 * 					TiDProxy::AsReaderWriter. False if it is not or a 
		 * 					connection to the given \a pipe does not exist
		 */
		bool IsWriter(std::string pipe);

		/*! \brief Check if the connection is set as reader
		 * 
		 * This method checks if the connection identified by the \a pipe name
		 * is set as reader (aka, ClTobiId::GetOnly or ClTobiId::SetGet).
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection mode is TiDProxy::AsReader or
		 * 					TiDProxy::AsReaderWriter. False if it is not or a 
		 * 					connection to the given \a pipe does not exist
		 */
		bool IsReader(std::string pipe);
		
		/*! \brief Check if the connection is set as reader/writer
		 * 
		 * This method checks if the connection identified by the \a pipe name
		 * is set as reader/writer (aka, ClTobiId::SetGet).
		 *
		 * \param 	pipe 	Identifier of the connection (via pipe name)
		 * \return 			True if the connection mode is TiDProxy::AsReaderWriter. 
		 * 					False if it is not or a connection to the given \a 
		 * 					pipe does not exist
		 */
		bool IsReaderWriter(std::string pipe);
		
	private:
		void onRunning(void);
		void onReceived(const cnbiros_bci::TiDMessage& msg);

	public:
		//! \brief AsReader modality for a connection (aka, ClTobiId::GetOnly)
		const static unsigned int AsReader = 0;
		
		//! \brief AsWriter modality for a connection (aka, ClTobiId::SetOnly)
		const static unsigned int AsWriter = 1;
		
		//! \brief AsReaderWriter modality for a connection (aka, ClTobiId::GetOnly)
		const static unsigned int AsReaderWriter = 2;

	private:
		std::map<std::string, ClTobiId*> 	id_map_;
		std::map<std::string, std::string>	pt_map_;
		std::map<std::string, unsigned int> pm_map_;



};


	}
}


#endif

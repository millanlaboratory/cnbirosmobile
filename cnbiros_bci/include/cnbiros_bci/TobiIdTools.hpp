#ifndef CNBIROS_BCI_TOBIID_TOOLS_HPP
#define CNBIROS_BCI_TOBIID_TOOLS_HPP

#include <tobiid/IDMessage.hpp>
#include "cnbiros_bci/TiDMessage.h"

namespace cnbiros {
	namespace bci {

//* TobiIdTools class
/** \brief Support class to manage conversion between IDMessage and
 * cnbiros_bci::TiDMessage.
 *
 * \par General description:
 * TobiIdTools is a support class to handle in an easy way the conversion
 * between IDMessage (from tobiid library) and cnbiros_bci::TiDMessage (in ROS
 * ecosystem). The class provides static methods to be used without
 * instantiation.
 *
 *	\todo 
 *	- Add new functionalities that replicate the IDMessage methods?
 *
 * \sa TiDProxy
 */
class TobiIdTools {
	
	public:
		//! \brief Constructor
		TobiIdTools(void);
		
		//! \brief Destructor
		~TobiIdTools(void);

		/*! \brief Method to convert from cnbiros_bci::TiDMessage to IDMessage
		 *
		 * This methods convert a cnbiros_bci::TiDMessage (ROS ecosystem) into a
		 * IDMessage (CNBI loop ecosystem). The resulting IDMessage can be
		 * serialized and sent to the loop via a ClTobiId object (or via
		 * TiDProxy). In this case, the block id of the message is set as
		 * undefined. The method is defined as static (possibility to be used
		 * without instantiation of the class).
		 *
		 * \param[in] 	in 		ROS message to be converted
		 * \param[out] 	out 	Resulting IDMessage
		 *
		 */
		static void GetMessage(const cnbiros_bci::TiDMessage& in, IDMessage& out);
		
		/*! \brief Method to convert from IDMessage to cnbiros_bci::TiDMessage
		 *
		 * This methods convert a IDMessage (CNBI loop ecosystem) into a
		 * cnbiros_bci::TiDMessage (ROS ecosystem). The resulting
		 * cnbiros_bci::TiDMessage can be publish to a ROS topic (e.g.,  via
		 * TiDProxy). The method is defined as static (possibility to be used
		 * without instantiation of the class).
		 *
		 * \param[in] 	in 		IDMessage to be converted
		 * \param[out] 	out 	Resulting ROS message
		 *
		 */
		static void GetMessage(const IDMessage& in, cnbiros_bci::TiDMessage& out);
};

	}
}

#endif

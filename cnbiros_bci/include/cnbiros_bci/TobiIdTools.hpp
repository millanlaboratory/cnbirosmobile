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
		TobiIdTools(const cnbiros_bci::TiDMessage& msg);
		
		TobiIdTools(const IDMessage& idm);
		
		//! \brief Destructor
		~TobiIdTools(void);

		/*! \brief Method to get the ID message in IDMessage (Tobi) format
		 *
		 * This methods convert the IDMessage (CNBI loop ecosystem)	into a
		 * cnbiros_bci::TiDMessage
		 *
		 * \param[out] 	idm 	Resulting IDMessage
		 *
		 */
		void GetMessage(IDMessage& idm);
		
		/*! \brief Method to get the ID message in cnbiros_bci::TiDMessage
		 * format
		 *
		 * This methods convert the cnbiros_bci::TiDMessage into IDMessage (CNBI
		 * loop ecosystem)		 
		 * 
		 * \param[out] 	msg 	Resulting ROS message
		 *
		 */
		void GetMessage(cnbiros_bci::TiDMessage& msg);

		bool IsFromPipe(const std::string& pipe);

	private:
		cnbiros_bci::TiDMessage 	rosmsg_;
};

	}
}

#endif

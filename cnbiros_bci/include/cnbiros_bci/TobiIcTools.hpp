#ifndef CNBIROS_BCI_TOBIIC_TOOLS_HPP
#define CNBIROS_BCI_TOBIIC_TOOLS_HPP

#include <unordered_map>
#include <tobiic/ICMessage.hpp>
#include "cnbiros_bci/TiCMessage.h"
#include "cnbiros_bci/TiCClassifier.h"
#include "cnbiros_bci/TiCClass.h"

namespace cnbiros {
	namespace bci {

//* TobiIcTools class
/** \brief Support class to manage conversion between ICMessage and
 * cnbiros_bci::TiCMessage.
 *
 * \par General description:
 * TobiIcTools is a support class to handle in an easy way the conversion
 * between ICMessage (from tobiic library) and cnbiros_bci::TiCMessage (in ROS
 * ecosystem).  
 * 
 *	\todo 
 *	- Same structure as TobiIdTools
 *	- Pointer based implementation
 *	- Add new functionalities that replicate the ICMessage methods?
 *
 * \sa TiCProxy
 */
class TobiIcTools {
	
	public:
		/*! \brief Constructor
		 *
		 * Constructor with the type of message to be converted
		 * 
		 * \todo Change it!
		 *
		 * \param 	msg 	ROS message
		 */
		TobiIcTools(const cnbiros_bci::TiCMessage& msg);
		
		/*! \brief Constructor
		 *
		 * Constructor with the type of message to be converted
		 * 
		 * \todo Change it!
		 *
		 * \param 	msg 	ICMessage
		 */
		TobiIcTools(const ICMessage& msg);
		~TobiIcTools(void);

		/*! \brief Conversion from cnbiros_bci::TiCMessage to ICMessage
		 *
		 * Method to convert the cnbiros_bci::TiCMessage provided in the
		 * constructor into a ICMessage
		 *
		 * \todo Change it!
		 *
		 * \param 	msg 	Message to be converted
		 * \return 			True if the conversion has been succeded
		 */
		bool GetMessage(ICMessage& msg);
		
		/*! \brief Conversion from ICMessage to cnbiros_bci::TiCMessage
		 *
		 * Method to convert the ICMessage provided in the
		 * constructor into a cnbiros_bci::TiCMessage
		 *
		 * \todo Change it!
		 *
		 * \param 	msg 	Message to be converted
		 * \return 			True if the conversion has been succeded
		 */
		bool GetMessage(cnbiros_bci::TiCMessage& msg);

		bool IsFromPipe(const std::string& pipe);

		bool HasClassifier(const std::string& name);

		bool HasClass(const std::string& name, const std::string& label);

		bool GetClassifier(const std::string& name, cnbiros_bci::TiCClassifier& icclassifier);

		bool GetClass(const std::string& name, const std::string& label, cnbiros_bci::TiCClass& icclass);

	private:
		std::unordered_map<std::string, ICClassifier>			classifier_set_;
		std::unordered_map<std::string, std::vector<ICClass>>	class_set_;

		cnbiros_bci::TiCMessage 	rosmsg_;
};

	}
}

#endif

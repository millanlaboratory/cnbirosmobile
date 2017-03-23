#ifndef CNBIROS_BCI_TOBIID_TOOLS_HPP
#define CNBIROS_BCI_TOBIID_TOOLS_HPP

#include <tobiid/IDMessage.hpp>
#include "cnbiros_bci/TiDMessage.h"

namespace cnbiros {
	namespace bci {

class TobiIdTools {
	
	public:
		TobiIdTools(void);
		~TobiIdTools(void);

		static void GetMessage(const cnbiros_bci::TiDMessage& in, IDMessage& out);
		static void GetMessage(const IDMessage& in, cnbiros_bci::TiDMessage& out);

};

	}
}

#endif

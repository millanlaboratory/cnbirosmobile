#ifndef CNBIROS_BCI_TOBIIC_TOOLS_HPP
#define CNBIROS_BCI_TOBIIC_TOOLS_HPP

#include <unordered_map>
#include <tobiic/ICMessage.hpp>
#include "cnbiros_bci/TiCMessage.h"

namespace cnbiros {
	namespace bci {

class TobiIcTools {
	
	public:
		TobiIcTools(const cnbiros_bci::TiCMessage& msg);
		TobiIcTools(const ICMessage& msg);
		~TobiIcTools(void);

		bool GetMessage(ICMessage& msg);
		bool GetMessage(cnbiros_bci::TiCMessage& msg);


	private:
		std::unordered_map<std::string, ICClassifier>			classifier_set_;
		std::unordered_map<std::string, std::vector<ICClass>>	class_set_;

		cnbiros_bci::TiCMessage 	rosmsg_;
		


};

	}
}

#endif

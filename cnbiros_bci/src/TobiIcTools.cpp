#ifndef CNBIROS_BCI_TOBIIC_TOOLS_CPP
#define CNBIROS_BCI_TOBIIC_TOOLS_CPP

#include "TobiIcTools.hpp"

namespace cnbiros {
	namespace bci {

TobiIcTools::TobiIcTools(const cnbiros_bci::TobiIc& msg) {
	
	std::vector<cnbiros_bci::TobiIcClassifier>::const_iterator itclf;
	std::vector<cnbiros_bci::TobiIcClass>::const_iterator itcls;
	ICClassifier* clf;
	ICClass*	  cls;

	for(itclf = msg.classifiers.begin(); itclf != msg.classifiers.end(); ++itclf) {

		// Create a new classifier
		clf = new ICClassifier((*itclf).name, (*itclf).description, 
								(*itclf).vtype, (*itclf).ltype);

		// Add a new classifier to the set
		this->classifier_set_.emplace((*itclf).name, *clf);

		// Create empty vector of ICClass for this classifier
		std::vector<ICClass> vec;
		
		// Iterate, create a new class and add to the vector
		for(itcls = (*itclf).classes.begin(); itcls != (*itclf).classes.end(); ++itcls) {
			cls = new ICClass((*itcls).label, (*itcls).value);	
			vec.emplace(vec.end(), *cls);
		}
		this->class_set_.emplace((*itclf).name, vec);
	}
	delete clf;
	delete cls;
}

TobiIcTools::TobiIcTools(const ICMessage& icm) {

	ICClassifierConstIter itclf;
	ICSetClassConstIter itcls;

	std::vector<cnbiros_bci::TobiIcClassifier> vclf;
	for(itclf = icm.classifiers.Begin(); itclf != icm.classifiers.End(); ++itclf) {
		
		cnbiros_bci::TobiIcClassifier cclf;
		std::vector<cnbiros_bci::TobiIcClass> vcls;
		
		for(itcls = itclf->second->classes.Begin(); itcls != itclf->second->classes.End(); ++itcls) {
			cnbiros_bci::TobiIcClass ccls;
			ccls.label = itcls->second->GetLabel();
			ccls.value = itcls->second->GetValue();
			vcls.emplace(vcls.end(), ccls); 
		}
	

		cclf.name    = itclf->first;	
		cclf.classes = vcls;

		vclf.emplace(vclf.end(), cclf);
		
	}
	
	
	this->rosmsg_.classifiers = vclf;
	
}

TobiIcTools::~TobiIcTools(void) {}

bool TobiIcTools::GetMessage(ICMessage* icm) {

	std::unordered_map<std::string, ICClassifier>::iterator itclf;	
	std::unordered_map<std::string, std::vector<ICClass>>::iterator itclv;	
	std::vector<ICClass>::iterator itcls;

	// Iterate along the stored classifiers
	for(itclf = this->classifier_set_.begin(); itclf != this->classifier_set_.end(); ++itclf) {

		// Select the vector of classes for the current classifier
		itclv = this->class_set_.find(itclf->first);

		if(itclv != this->class_set_.end()) {
			// Iteratate along the classes of the current classifier
			for(itcls = (itclv->second).begin(); itcls != (itclv->second).end(); ++itcls) {
				// Add class references to the ICClassifier
				itclf->second.classes.Add(&(*itcls));
			}
		}
		// Add current classifier reference to the ICMessage
		icm->classifiers.Add(&(itclf->second));
	}
}

bool TobiIcTools::GetMessage(cnbiros_bci::TobiIc& msg) {

	//for(auto c = this->rosmsg_.classifiers.begin(); c != this->rosmsg_.classifiers.end(); ++c) {
	//	printf("classifier: %s\n", (*c).name.c_str());
	//	
	//	for(auto i = (*c).classes.begin(); i != (*c).classes.end(); ++i) { 
	//		printf("|- label: %s, value: %f\n", (*i).label.c_str(), (*i).value);
	//	}
	//}
	msg = this->rosmsg_;
}


	}
}

#endif

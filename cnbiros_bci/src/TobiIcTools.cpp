#ifndef CNBIROS_BCI_TOBIIC_TOOLS_CPP
#define CNBIROS_BCI_TOBIIC_TOOLS_CPP

#include "cnbiros_bci/TobiIcTools.hpp"

namespace cnbiros {
	namespace bci {

TobiIcTools::TobiIcTools(const cnbiros_bci::TiCMessage& msg) {
	
	std::vector<cnbiros_bci::TiCClassifier>::const_iterator itclf;
	std::vector<cnbiros_bci::TiCClass>::const_iterator itcls;
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

	std::vector<cnbiros_bci::TiCClassifier> vclf;
	for(itclf = icm.classifiers.Begin(); itclf != icm.classifiers.End(); ++itclf) {
		
		cnbiros_bci::TiCClassifier cclf;
		std::vector<cnbiros_bci::TiCClass> vcls;
		
		for(itcls = itclf->second->classes.Begin(); itcls != itclf->second->classes.End(); ++itcls) {
			cnbiros_bci::TiCClass ccls;
			ccls.label = itcls->second->GetLabel();
			ccls.value = itcls->second->GetValue();
			vcls.emplace(vcls.end(), ccls); 
		}
	

		cclf.name    	 = itclf->first;	
		cclf.description = itclf->second->GetDescription(); 
		cclf.vtype  	 = itclf->second->GetValueType();
		cclf.ltype 		 = itclf->second->GetLabelType();
		cclf.classes = vcls;

		vclf.emplace(vclf.end(), cclf);
		
	}

	this->rosmsg_.frame = icm.GetBlockIdx();	
	this->rosmsg_.classifiers = vclf;
}

TobiIcTools::~TobiIcTools(void) {}

bool TobiIcTools::GetMessage(ICMessage& icm) {

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
		icm.classifiers.Add(&(itclf->second));
	}
}

bool TobiIcTools::GetMessage(cnbiros_bci::TiCMessage& msg) {
	msg = this->rosmsg_;
}

bool TobiIcTools::HasClassifier(const std::string& name) {

	bool has_classifier = false;

	for(auto it=this->rosmsg_.classifiers.begin(); it!=this->rosmsg_.classifiers.end(); ++it) {
		if((*it).name.compare(name) == 0) {
			has_classifier = true;
			break;
		}
	}

	return has_classifier;
}

bool TobiIcTools::GetClassifier(const std::string& name, cnbiros_bci::TiCClassifier& classifier) {
	
	if(this->HasClassifier(name) == false) {
		return false;
	}

	for(auto it=this->rosmsg_.classifiers.begin(); it!=this->rosmsg_.classifiers.end(); ++it) {
		if((*it).name.compare(name) == true) {
			classifier = (*it);
			break;
		}
	}

	return true;
}

bool TobiIcTools::HasClass(const std::string& name, const std::string& label) {

	cnbiros_bci::TiCClassifier classifier;
	bool has_class = false;

	if(this->GetClassifier(name, classifier) == false) {
		return has_class;
	}

	for(auto it = classifier.classes.begin(); it!=classifier.classes.end(); ++it) {
		if((*it).label.compare(label) == 0) {
			has_class = true;
			break;
		}
	}

	return has_class;
}

bool TobiIcTools::GetClass(const std::string& name, const std::string& label, 
						   cnbiros_bci::TiCClass& icclass) {

	cnbiros_bci::TiCClassifier classifier;

	if(this->GetClassifier(name, classifier) == false) {
		return false;
	}

	for(auto it = classifier.classes.begin(); it!=classifier.classes.end(); ++it) {
		if((*it).label.compare(label) == 0) {
			icclass = (*it);
			break;
		}
	}

	return true;
}


	}
}

#endif

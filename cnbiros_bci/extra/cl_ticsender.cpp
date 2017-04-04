#include <cnbiloop/ClLoop.hpp>
#include <cnbiloop/ClTobiIc.hpp>
#include <getopt.h>
#include <cmath>
void usage(void) { 
	printf("Usage: cl_ticsender [OPTION]...\n\n");
	printf("  -a       address of nameserver in ip:port format\n");
	printf("  -p       TCP port (9500 default)\n");
	printf("  -n       TCP client name (/ctrl10 default)\n");
	printf("  -c       Classifier name (mobile default)\n");
	printf("  -l       Class label (control default)\n");
	printf("  -d       Dump message sent\n");
	printf("  -h       display this help and exit\n");
}

float sinewave(float a, float dt, float f) {
	return a*sin((2.0f*M_PI*dt*f));
}

int main(int argc, char* argv[]) {
	int opt;
	std::string optname("/ctrl10");
	std::string optclass("mobile");
	std::string optlabel("control");
	CcPort optport("");
	CcAddress nameserver;
	bool dump = false;
	
	while((opt = getopt(argc, argv, "a:p:n:hd")) != -1) {
		if(opt == 'p')
			optport.assign(optarg);
		else if(opt == 'n')
			optname.assign(optarg);
		else if(opt == 'c')
			optclass.assign(optarg);
		else if(opt == 'l')
			optlabel.assign(optarg);
		else if(opt == 'd')
			dump = true;
		else if(opt == 'a')
			nameserver.assign(optarg);
		else {
			usage();
			CcCore::Exit(opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE);
		}
	}

	CcCore::OpenLogger("cl_ticsender");
	CcCore::CatchSIGINT();
	CcCore::CatchSIGTERM();
	ClLoop::Configure(nameserver);

	// Initialization IC Message
	ICMessage* 			icm;
	ICSerializerRapid* 	ics;
	ICClass* 			icclass;
	ICClassifier* 		icclassifier;

	// Initialization ClTobiIC interface
	ClTobiIc* ic;

	// Initialization support variables
	std::string absolute, relative;
	bool 	attached;
	int 	status, block;
	float 	value;
	float 	dt;
	CcTimeValue start;

	// Creation IC Message	
	icm = new ICMessage;
	ics = new ICSerializerRapid(icm);

	icclass = 	   new ICClass(optlabel, 0.0f);
	icclassifier = new ICClassifier(optclass, "Sinusoid classifier at 0.5 Hz",
					ICClassifier::ValueProb, ICClassifier::LabelCustom);

	icclassifier->classes.Add(icclass);
	icm->classifiers.Add(icclassifier);

	ic = new ClTobiIc(ClTobiIc::SetOnly);

	// Connection to CNBI Loop
	if(ClLoop::Connect() == false) {
		CcLogFatal("Cannot connect to loop");
		CcCore::Exit(EXIT_FAILURE);
	}

	// Connection to ClTobiIc
	attached = false;
	CcLogInfoS("Waiting for connection at " << optname <<":"<<optport);
	while(attached == false) {
		attached = optport.empty() ? ic->Attach(optname) : ic->Attach(optport, optname);
		CcTime::Sleep(1000.0f);
	}
	CcLogInfoS("Connected at " << optname <<":"<<optport);


	dt = 0;
	block = 0;
	CcTime::Tic(&start);

	while(true) {
		icm->SetBlockIdx(block++);

		value = sinewave(1.0f, CcTime::Toc(&start)/1000.0f, 0.5f);

		icm->SetValue(optclass, optlabel, value);

		if(ic->SetMessage(ics) == ClTobiIc::Detached) {
			CcLogFatal("Tobi Ic detached");
			break;
		}
		
		if(dump == true)
			icm->Dump();
		
		CcTime::Sleep(10.0f);

		if(CcCore::receivedSIGAny.Get()) 
			break;
	}


	ic->Detach();

	delete ic;
	delete icclass;
	delete icclassifier;
	delete icm;
	delete ics;

	CcCore::Exit(0);
}

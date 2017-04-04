#include <cnbiloop/ClLoop.hpp>
#include <cnbiloop/ClTobiIc.hpp>
#include <getopt.h>
#include <cmath>
void usage(void) { 
	printf("Usage: cl_ticsender [OPTION]...\n\n");
	printf("  -a       address of nameserver in ip:port format\n");
	printf("  -p       TCP port (9500 default)\n");
	printf("  -n       TCP client name (/ctrl0 default)\n");
	printf("  -d       Dump message sent\n");
	printf("  -h       display this help and exit\n");
}

float sinewave(double time, double frequency) {
	return sin((2*M_PI)*time*frequency/1000.0f);
}

//! \todo To clean and review
int main(int argc, char* argv[]) {
	int opt;
	std::string optname("/ctrl0");
	CcPort optport("");
	CcAddress nameserver;
	bool locking = false, dump = false;
	
	while((opt = getopt(argc, argv, "a:p:n:hd")) != -1) {
		if(opt == 'p')
			optport.assign(optarg);
		else if(opt == 'n')
			optname.assign(optarg);
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
	ICMessage icm;
	ICSerializerRapid ics(&icm);

	ICClass k1("0x100", 0.0f);

	ICClassifier c("classifier", "Sinusoid classifier at 10 Hz",
					ICClassifier::ValueProb, ICClassifier::LabelCustom);

	c.classes.Add(&k1);

	icm.classifiers.Add(&c);

	ClTobiIc ic(ClTobiIc::SetOnly);
	std::string absolute, relative;
	bool attached;
	int status;
	
	if(ClLoop::Connect() == false) {
		CcLogFatal("Cannot connect to loop");
		CcCore::Exit(EXIT_FAILURE);
	}
	
	attached = false;
	CcLogInfoS("Waiting for connection at " << optname <<":"<<optport);
	while(attached == false) {
		attached = optport.empty() ? ic.Attach(optname) : ic.Attach(optport, optname);
		CcTime::Sleep(1000.0f);
	}
	CcLogInfoS("Connected at " << optname <<":"<<optport);


	int block = 0;
	float value;

	CcTimeValue start;
	CcTime::Tic(&start);

	unsigned int i = 0;
	while(true) {
		icm.SetBlockIdx(block++);

		value = sin(0.5*(2.0f*M_PI)*i/20.0f);

		icm.SetValue("classifier", "0x100", value);

		i++;

		if(ic.SetMessage(&ics) == ClTobiIc::Detached) {
			CcLogFatal("Tobi Ic detached");
			break;
		}
		
		if(dump == true)
			icm.Dump();
		
		CcTime::Sleep(50.0f);

		if(CcCore::receivedSIGAny.Get()) 
			break;
	}


	ic.Detach();
	CcCore::Exit(0);
}

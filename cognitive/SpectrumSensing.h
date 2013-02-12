// CRAHNs Model START
// @author:  Marco Di Felice

#ifndef SPECTRUM_SENSING_H
#define SPECTRUM_SENSING_H

#include <mobilenode.h>

#include "PUmodel.h"
#include "SpectrumManager.h"

#include "repository.h" //LI_MOD

class SpectrumManager;

// Implementation of the Spectrum Sensing activity performed by CRs
class SpectrumSensing {
	
	public:
		
		// Initialitation method: PUmodel off
		SpectrumSensing(SpectrumManager *sm);

		// Initialization method: PUmodel on
		SpectrumSensing(SpectrumManager *sm, double prob, PUmodel *p);
	
		// Perform sensing and return true if PU activity is detected on the current channel
		bool sense(int id, double sense_time, double transmit_time, int channel);

		#ifdef LI_MOD
		bool sense_prev_tx(int id, double sense_time, double transmit_time, int channel);
		void sense_all_channels(int id, double sense_time, Repository *repo);
		#endif
	
	private:
		
		// Primary User Map and Model
		PUmodel *pumodel_;
		
		// Spectrum Manager reference
		SpectrumManager *smanager_;

		// Probability to have false negative detection of PUs.
		double prob_misdetect_;

		#ifdef LI_MOD
		// Counter of sensing times
		int sense_counter;
		#endif
};


#endif


// CRAHNs Model END
// @author:  Marco Di Felice



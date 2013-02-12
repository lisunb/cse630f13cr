// CRAHNs Model START
// @author:  Marco Di Felice


#ifndef SPECTRUM_DATA_H
#define SPECTRUM_DATA_H


#include <cognitive/repository.h>
#include <stdio.h>
#include "object.h"

// Verbose mode
#define IO_SPECTRUM_DEBUG 1


// Spectrum Entry Information
struct spectrum_entry_t  {
	
	// current bandwidth
	double bandwidth;
	
#ifndef LI_MOD // No LI_MOD
	// Packet Error Rate (PER) value	
	double per;
#endif

#ifdef LI_MOD // LI_MOD
	// per [(receiver)] [(sender)]
	double per[MAX_NODES][MAX_NODES];
#endif

 };


class SpectrumData : public NsObject {

	public:
		
		// SpectrumData Initializer
		SpectrumData();
		
		// Return the spectrum-entry information for the current channel
		spectrum_entry_t get_spectrum_data(int channel);
	
		// command method 
		int command(int argc, const char*const* argv);
		
		// recv method (EMPTY)
		void recv(Packet*, Handler*); 

#ifdef LI_MOD
		spectrum_entry_t spectrum_table_[MAX_CHANNELS];
#endif // LI_MOD
		
	private:
		
		// read the current spectrum file, and load the information in the spectrum_table_
		void read_spectrum_file(char *fileName);
		
#ifndef LI_MOD
		// Spectrum_table, loaded by the spectrum file
		spectrum_entry_t spectrum_table_[MAX_CHANNELS];
#endif // No LI_MOD

};


#endif

// CRAHNs Model END
// @author:  Marco Di Felice




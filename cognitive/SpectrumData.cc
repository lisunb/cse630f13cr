// CRAHNs Model START
// @author:  Marco Di Felice



#include "SpectrumData.h"


static class SpectrumMapclass : public TclClass {
public:
        SpectrumMapclass() : TclClass("SpectrumMap") {}
        TclObject* create(int argc, const char*const* argv) {
          return (new SpectrumData());
        }
} class_spectrum_data;




// SpectrumData initializer
SpectrumData::SpectrumData()  {

}



// read_spectrum_file: load the information form the spectrum file into the spectrum_table_
void
SpectrumData::read_spectrum_file(char *fileName) {
	
	FILE* fd;	
	
	fd=fopen(fileName,"rt");	
	
	if (IO_SPECTRUM_DEBUG) 
		printf("Reading PU Data from File: %s \n", fileName);
	

	if (fd==NULL) {
		printf(" ERROR. Can't open file %s \n",fileName);
		exit(0);
	}


	// For each channel in the range [0: MAX_CHANNELS]
	// the spectrum file contains these entries:
	// - bandwidth (b/s)
	// - packet error rate 
	
#ifndef LI_MOD
	for (int i=0; i<MAX_CHANNELS; i++)  {

		int channel;
		float bandwidth;
		float per;
	
		// read the next entry
		fscanf(fd,"%d %f %f",&channel,&bandwidth,&per);
		
		if (ferror(fd)) {
			printf(" ERROR. Can't read Spectrum Information from file %s \n", fileName);		
			exit(0);
		}


		if (IO_SPECTRUM_DEBUG)
			printf("[READING SPECTRUM FILE] #CHANNEL: %d #BANDWIDTH: %f PER: %f\n",channel, bandwidth, per); 

		// save the information in the spectrum_table_
		spectrum_table_[channel].bandwidth=bandwidth;
		spectrum_table_[channel].per=per;
	 }
#endif // No LI_MOD	

#ifdef LI_MOD
	int entry_num = (MAX_CHANNELS*MAX_NODES*MAX_NODES);

	for (int i=0; i < entry_num; i++) {

		int rx;
		int tx;
		int channel;
		float bandwidth;
		float per;
	
		// read the next entry
		fscanf(fd, "%d %d %d %f %f", &channel, &rx, &tx, &bandwidth, &per);
		
		if (ferror(fd)) {
			printf(" ERROR. Can't read Spectrum Information from file %s \n", fileName);		
			exit(0);
		}

		#ifdef LI_DEBUG
		if (IO_SPECTRUM_DEBUG)
			printf("[READING SPECTRUM FILE] #CHANNEL: %d #RX: %d #TX: %d #BANDWIDTH: %f PER: %f\n",
					channel, rx, tx, bandwidth, per); 
		#endif // LI_DEBUG
	
		// save the information in the spectrum_table_
		spectrum_table_[channel].bandwidth=bandwidth;
		spectrum_table_[channel].per[rx][tx]=per;
	 }
#endif // LI_MOD
	
	 fclose(fd);
}




// get_spectrum_data: return the spectrum_entry for the current channel
spectrum_entry_t 
SpectrumData::get_spectrum_data(int channel) {

	if ((channel>=0) && (channel <MAX_CHANNELS)) 
		return spectrum_table_[channel];

	else  {
		printf(" ERROR. Can't retrive Spectrum Information for channel %d \n", channel);		
		exit(0);
	
	 }
	
}



// comman: get input from OTCL file
int 
SpectrumData::command(int argc, const char*const* argv) {
 	
	if(argc == 3) {
		
		// Read the current spectrum file		
		if(strcmp(argv[1], "set_input_map") == 0) {
  		    read_spectrum_file((char*)argv[2]);
   		    return TCL_OK;
		
		}
	
	} 

	return TCL_OK;
}




// recv method: Receive a pkt (EMPTY METHOD)
void 
SpectrumData::recv(Packet*, Handler*) {

}



// CRAHNs Model START
// @author:  Marco Di Felice



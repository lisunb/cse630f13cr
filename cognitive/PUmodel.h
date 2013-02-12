// pu_activity.h

// A model for PU activity and CR channel sensing

#ifndef NS_PU_MODEL_H
#define NS_PU_MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random.h>
#include "object.h"


// Constant value for the PU Mapping file
// Max number of PUs 
//# define MAX_PU_USERS 		60
# define MAX_PU_USERS 		120
// Max number of PU channel spectrum
# define MAX_CHANNEL 		11
// Max number of PU-data entry
//# define MAX_PU_DATA_ENTRY  	700
# define MAX_PU_DATA_ENTRY  	700
// Debug variable: enable verbose mode
# define IO_DEBUG		1


//PU information
typedef struct pu_activity {
	// channel used for tx
	int main_channel;
	// number of arrival/departure entries
	int number_data;
	// current location
	double x_loc;
	double y_loc;
	// arrival/departure time
	double arrival_time[MAX_PU_DATA_ENTRY];
	double departure_time[MAX_PU_DATA_ENTRY];
	bool detected[MAX_PU_DATA_ENTRY];
	// PU receiver location
	double x_loc_receiver;	
	double y_loc_receiver;	
	// PU <alpha-beta> activity description
	double alpha;
	double beta;
	// PU transmitting range
	double radius;
	// Avg. interference caused by CR on the PU receiver
	double interference;
};


class PUmodel : public NsObject {

	public:	
		
		// PUmodel creator
		PUmodel();
		// Method for receiving command from OTCL
		int command(int argc, const char*const* argv);
		// Receiving packet method (NOT used)
		void recv(Packet*, Handler*); 	
		// Return true if a PU is transmitting in the same spectrum of the CR
		bool is_PU_active(double timeNow, double ts, double x, double y, int channel);
		// Write the statistics about interference on PU receivers
		void write_stat(int param);
		// Check if the tranmission of a CR may cause interference to a PU receiver
		void update_stat_pu_receiver(int id, double timeNow, double txtime, double x, double y, int channel);

	private:

		// Number of PUs in the current scenario
		int number_pu_;
		// Data structures with information of PUs
		pu_activity pu_data[MAX_PU_USERS];
		// Method to read data from PU file and save them in the pu_activity data structure 
		void read_data(char * dir);
		// Method to get the distance from the PU transmitter
		double distance(double x, double y, int channel);
		// Method to get the distance from the PU receiver
		double distance_receiver(double x, double y, int channel);
		// Method tc check if a PU is transmitting on a given spectrum at a given time
		bool check_active(double timeNow, double ts, int channel);
		
		// PU-Receiver interference statistics
		int interference_events_;
		
		double interference_power_;
};

#endif

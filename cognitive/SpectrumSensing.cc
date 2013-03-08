// CRAHNs Model START
// @author:  Marco Di Felice



#include "SpectrumSensing.h"


// SpectrumSensing initialization: PU model off
SpectrumSensing::SpectrumSensing(SpectrumManager *sm) {
 	
	smanager_=sm;

	#ifdef LI_MOD
	sense_counter  = 0;
	#endif
}




// SpectrumSensing initialization: PU model on
SpectrumSensing::SpectrumSensing(SpectrumManager *sm, double prob_misdetect, PUmodel *p) {
	
	pumodel_=p;

	prob_misdetect_=prob_misdetect;

#ifdef LI_MOD
	sense_counter  = 0;
#endif
}               




// sense: return true if PU activity is detected in the time interval [current_time:current_time + sense_time]
bool
SpectrumSensing::sense(int id, double sense_time, double transmit_time, int channel) {
	
	MobileNode *pnode = (MobileNode*)Node::get_node_by_address(id);
        double x=pnode->X();
        double y=pnode->Y();
	bool cr_on=false;

	if (pumodel_) {

		double randomValue=Random::uniform();

		// Ask the PUmodel if a PU is active  in the time interval [current_time:current_time + sense_time]
		cr_on=pumodel_->is_PU_active(Scheduler::instance().clock(),sense_time,x,y, channel);

		// Apply the probability of false negative detection
		if ((randomValue < prob_misdetect_) and cr_on)
			cr_on=false;

	}

	//#ifdef SENSING_VERBOSE_MODE
	//	printf("[SENSING-DBG] Node %d sensed pu activity on channel %d at time %f\n", id, channel, Scheduler::instance().clock());
	//#endif

	return cr_on;	
		
}

#ifdef LI_MOD
// return true if PU activity is detected in the time interval [current_time - sense_time:current_time]
bool
SpectrumSensing::sense_prev_tx(int id, double sense_time, double transmit_time, int channel) {
	
	MobileNode *pnode = (MobileNode*)Node::get_node_by_address(id);
	double x=pnode->X();
	double y=pnode->Y();
	bool cr_on=false;

	if (pumodel_) {

		double randomValue=Random::uniform();

		double sense_start, time_now;
		time_now = Scheduler::instance().clock();
		sense_start = time_now - sense_time;

		// Important! sensing time interval [current_time - sense_time:current_time]
		cr_on=pumodel_->is_PU_active(sense_start, sense_time, x, y, channel);

		if ((randomValue < prob_misdetect_) and cr_on)
			cr_on=false;
	}

	return cr_on;	
}

// sense_all_channels: sense each channel and update repository
void
SpectrumSensing::sense_all_channels(int id, double sense_time, Repository *repo) {

	MobileNode *pnode = (MobileNode*)Node::get_node_by_address(id);
        double x=pnode->X();
        double y=pnode->Y();

	if (pumodel_) {

		// No sensing error model is used here
		for(int i = 1; i < MAX_CHANNELS; i++) {
 
			if(pumodel_->is_PU_active(Scheduler::instance().clock(), sense_time, x, y, i)) { 
			// Channel i is found used by a PU
				// Mark this channel as false
				repo->mark_channel(id, i, false);
				// Update the number of PU appearance on that channel 
				repo->update_active_count(id, i);
				#ifdef CRP
				repo->update_nvs_table(id, sense_counter, i, true);
				#endif // CRP
			}
			else {
			// Channel i is not used by a PU
				// Channel i is available
				repo->mark_channel(id, i, true);
				#ifdef CRP
				repo->update_nvs_table(id, sense_counter, i, false);
				#endif // CRP
			}
				
		}

		//update channel utilities 
		sense_counter++;
		repo->update_channel_utility(id, sense_counter);

		// show channel utilities
		if (sense_counter == 300 || sense_counter == 500)
			repo->show_channel_utility(id);
	}

}
#endif //LI_MOD
                

// CRAHNs Model END
// @author:  Marco Di Felice

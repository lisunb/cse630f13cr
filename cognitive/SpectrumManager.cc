
// CRAHNs Model END
// @author:  Marco Di Felice


#include "SpectrumManager.h"

#define CURRENT_TIME    Scheduler::instance().clock()

//SpectrumManager Initializer
SpectrumManager::SpectrumManager(Mac802_11 *mac, int id): stimer_(this), ttimer_(this) {

	mac_=mac;
	nodeId_=id;
	
	// State Initialization
	pu_on_rx=false;	
	sensing_=false;

	// Spectrum Module Definition
	sensingMod_=new SpectrumSensing(this);
	decisionMod_=new SpectrumDecision(this);
	mobilityMod_=new SpectrumMobility(this);



}

//SpectrumManager Initializer
SpectrumManager::SpectrumManager(Mac802_11 *mac, int id, double sense_time, double transmit_time): stimer_(this), ttimer_(this)  {

	mac_=mac;
	nodeId_=id;
	pu_on_rx=false;
	sensing_=false;	
	
	// State Initialization
	sense_time_=sense_time;
	transmit_time_=transmit_time;
	
	// Spectrum Module Definition
	sensingMod_=new SpectrumSensing(this);
	decisionMod_=new SpectrumDecision(this);
	mobilityMod_=new SpectrumMobility(this);

}

/*
 * start: CR starts sensing on the current channel
 */

void
SpectrumManager::start() {
	
	// Retrive the current channel on which the CR is tuned on the RECEIVER interface
	int current_channel = repository_->get_recv_channel(nodeId_);

#ifdef LI_MOD // LI_MOD
	pu_on_tx=0;

	mac_->load_spectrum(current_channel);
	// set spectrum data pointer only once
	if (repository_->is_sd_pointer_set() == false) 
		repository_->set_sd_pointer(dataMod_);	
#else 
	// Load spectrum characteristics (bandwidth, PER, ...)
	mac_->load_spectrum(dataMod_->get_spectrum_data(current_channel));
#endif

	// Start sensing on the current channel for a sense_time_ interval
	stimer_.start(sense_time_);

}


/* 
 * is_channel_available: return true if CR is NOT doing sensing and is NOT doing spectrum handoff
 */

bool 
SpectrumManager::is_channel_available() {
	
	bool available= !(sensing_ || mobilityMod_->is_switching());

	return available;

}



/*
 * is_PU_interfering: return true if there is a PU which is transmitting on the same channel and 
 * within the tx range of the CR receiving a packet
 */

bool 
SpectrumManager::is_PU_interfering(Packet *p) {

	// Get the tx time of a packet
	double time_tx=HDR_CMN(p)->txtime();
	// Check if a PU is active in the interval [now: now+time_tx]
	int  current_channel=repository_->get_recv_channel(nodeId_);
	bool interference=sensingMod_->sense(nodeId_,time_tx,transmit_time_, current_channel);
	
	#ifdef SENSING_VERBOSE_MODE
	if (interference)
		printf("[SENSING-DBG] Node %d sensed some PU activity on channel %d while receiving data\n", nodeId_,current_channel); 
	#endif

	return interference;
}

#ifdef LI_MOD
bool 
SpectrumManager::is_PU_interfering_ACK(Packet *p, int channel) {
	double time_tx=HDR_CMN(p)->txtime();
	bool interference=sensingMod_->sense(nodeId_,time_tx,transmit_time_, channel);
	return interference;
}
#endif

/*********************************************************
 * SETUP METHODS
 * *******************************************************/

//setPUmodel: set the current PU model
void
SpectrumManager::setPUmodel(double prob, PUmodel *p) {

	sensingMod_=new SpectrumSensing(this,prob,p);

}

//setRepository: set the current cross-layer repository
void
SpectrumManager::setRepository(Repository* rep) {
	
	repository_=rep;
}

//setSpectrumData: set the current Spectrum Loader module
void 
SpectrumManager::setSpectrumData(SpectrumData *sd) {
	
	dataMod_=sd;

}


/*********************************************************
 * TIMER METHODS
 * *******************************************************/

/*
 * senseHandler: handler for sensing timer. 
 * Check if PU was detected during the last sensing interval, in case ask the spectrumDecision to switch to a new channel.
 * In case of channel switching, use Spectrum Mobility to perform handoff, and notify the event to the upper layers.
 */

void 
SpectrumManager::senseHandler() {

	int current_channel = repository_->get_recv_channel(nodeId_);
	int next_channel = -1;
	
	// Error Check
	if(current_channel == 0) {
		printf("[!!!WARNING!!!] node: %d is using control channel.\n", nodeId_);
		exit(0);
	}
	
		#ifdef SENSING_VERBOSE_MODE //abdulla
		printf("[SENSING-DBG] Node %d is on channel %d and PU activity is %s at time: %f\n", 
				nodeId_, current_channel, (pu_on_rx)?"true":"false", Scheduler::instance().clock());
		#endif

	// whether a relay node or how many prev-hop TX
	int num_prev_relay = repository_->check_rx_set(nodeId_); // whether relay node

	// switch channel randomly even pu didn't show up - this benefits samer - Li
	if (num_prev_relay == 1) { 
		if (pu_on_rx) { // PU detected and this is a relay node 
			for(int i=0; i < num_prev_relay; i++)
				printf("\n [PU Shows Up!] Node: %d Current_Channel: %d Time: %f Set_Times: %d\n", 
						nodeId_, current_channel, CURRENT_TIME, num_prev_relay);
		}
		else { // look for a better channel after some time - Li
			double randomValue = Random::uniform();
			if(randomValue < 0.25) {
				printf("\n [SU Adapts Channel!] Node: %d Current_Channel: %d Time: %f\n", 
						nodeId_, current_channel, CURRENT_TIME);
				pu_on_rx = true; 
			}
		}
	}

	// Find prev-hop nodes and check tx and rx channels
	if (num_prev_relay != 0) { 

		for(int i=0; i < MAX_FLOWS; i++) { // Initialization
			prev_hop[i].id=-1;
			prev_hop[i].flow=-1;
			prev_hop[i].pu_on=false;
		}

		// Get prev-hop node id
		int node_list[MAX_FLOWS+1];
		node_list[0] = nodeId_;
		
		int counter_=0;
		for(int i=0; i < MAX_FLOWS; i++) {
			int flow_ = repository_->read_flow_id(nodeId_, i);
			if(flow_ != -1) {
				prev_hop[counter_].flow = flow_;
				prev_hop[counter_].id = repository_->find_prev_hop(nodeId_, flow_);
				node_list[counter_+1] = prev_hop[counter_].id;
				counter_++;
			}
		}

		// Error Check
		if(counter_ != num_prev_relay) {
			printf("[!!!WARNING!!!] set num is not correct on node %d\n", nodeId_);
			exit(0);
		}

		// Sense each prev-hop TX 
		pu_on_tx=0;
		int uflow_list[MAX_FLOWS];
		int uflow_list_all[MAX_FLOWS];
		int udst_list[MAX_FLOWS]; 
		int udst_list_all[MAX_FLOWS]; // all flows' dst 

		for(int i=0; i < num_prev_relay; i++) {
			uflow_list_all[i] = prev_hop[i].flow;
			udst_list_all[i] = repository_->read_flow_dst(prev_hop[i].flow);

			prev_hop[i].pu_on = sensingMod_->sense_prev_tx(prev_hop[i].id, sense_time_, transmit_time_, current_channel);
			if(prev_hop[i].pu_on) {
				uflow_list[pu_on_tx] = prev_hop[i].flow;
				udst_list[pu_on_tx] = repository_->read_flow_dst(prev_hop[i].flow);
				pu_on_tx++;
			}
		}

		// RX detected PU or TX detected PU
		if( pu_on_rx == true || pu_on_tx != 0 ) { 

			mobilityMod_->performHandoff(); // Starts handoff timer

			// Find the best available channel accroding to routing metric - Li
			#ifdef CHANNEL_DECISION_MAC_LAYER // Actually, not at MAC Layer - Li 
			next_channel = repository_->change_channel(node_list, (num_prev_relay+1), CURRENT_TIME);

			// Error Check 
			if(next_channel == 0) {
				printf("[!!!WARNING!!!] node: %d is using control channel.\n", nodeId_);
				exit(0);
			}

			// can't find any available channel for all
			if( next_channel == -1 && pu_on_rx == true) {

				// limark
				printf(" clean_all_route_channel:\n");
				for(int i=0; i < num_prev_relay; i++) {
					printf("ure dst %d\n", udst_list_all[i]);
				}

				repository_->clean_route_channel(uflow_list_all, num_prev_relay);
				mac_->notifyUpperLayer(udst_list_all, num_prev_relay);
			}
			else if( next_channel == -1 && pu_on_rx == false) {

				// limark
				printf(" clean_route_channel:\n");
				printf(" all dst:");
				for(int i=0; i < num_prev_relay; i++) {
					 printf(" %d", udst_list_all[i]);
				}
				printf("\n"); 
				for(int i=0; i < pu_on_tx; i++) {
					printf("ure dst %d\n", udst_list[i]);
				} 

				repository_->clean_route_channel(uflow_list, pu_on_tx);
				mac_->notifyUpperLayer(udst_list, pu_on_tx);
			}
			// We find some channels that can be used - Li
			else { 
				
				printf(" [SU Changes Channel!] Node: %d Current_Channel: %d Next_Channel: %d Time: %f\n\n", 
						nodeId_, current_channel, next_channel, CURRENT_TIME);

				repository_->set_recv_channel(nodeId_, next_channel);
				mac_->load_spectrum(next_channel); // New one - Li
			}

			#endif // CHANNEL_DECISION_MAC_LAYER

			#ifdef SENSING_VERBOSE_MODE
			printf("[SENSING-DBG] Node %d starts handoff on channel %d to channel %d at time %f \n",
					nodeId_,current_channel,next_channel,Scheduler::instance().clock()); 
			#endif
			
		} // end of if( pu_on_rx == true || pu_on_tx != 0 ) 
		else {
			pu_on_rx = false;
			ttimer_.start(transmit_time_);
			mac_->checkBackoffTimer(); // Start the backoff timer
		}
	} // end of if(num_prev_relay != 0) 
	else {
		pu_on_rx = false;
		ttimer_.start(transmit_time_);
		mac_->checkBackoffTimer(); // Start the backoff timer

		#ifdef SENSING_VERBOSE_MODE
		printf("[SENSING-DBG] Node %d starts transmitting on channel %d at time %f \n",
				nodeId_,current_channel,Scheduler::instance().clock()); 
		#endif
	}
	
	sensing_=false;
}

//transmitHandler: the CR stops transmitting, and starts sensing for PU detection
void 
SpectrumManager::transmitHandler() {

	int current_channel = repository_->get_recv_channel(nodeId_); 
	// Perform sensing on the current channel
	pu_on_rx = sensingMod_->sense(nodeId_,sense_time_,transmit_time_, current_channel);
	
#ifdef LI_MOD
	// Sense all channels and update the repository
	sensingMod_->sense_all_channels(nodeId_, sense_time_, repository_);
#endif 

	sensing_=true;
	// Start the sensing interval
	stimer_.start(sense_time_);


		#ifdef SENSING_VERBOSE_MODE
		printf("[SENSING-DBG] Node %d starts sensing on channel %d at time %f \n",
				nodeId_,current_channel,Scheduler::instance().clock()); 
		//if (pu_on_rx) printf("[SENSING-DBG] Node %d sensed pu activity on channel %d \n", nodeId_, current_channel);
		#endif

	// Stop any current backoff attempt
	mac_->checkBackoffTimer();
}

//endHandoff: the CR has performed spectrum handoff to a new channel. Then, it starts sensing on it to detect PU activity.
void 
SpectrumManager::endHandoff() {
	
#ifndef LI_MOD // no LI_MOD
	int current_channel=repository_->get_recv_channel(nodeId_);
	// Perform sensing on the new channel
	pu_on_rx = sensingMod_->sense(nodeId_,sense_time_,transmit_time_, current_channel);
	// Start the sensing interval
	stimer_.start(sense_time_);

		#ifdef SENSING_VERBOSE_MODE
		printf("[SENSING-DBG] Node %d ends handoff on channel %d at time %f \n",
				nodeId_,current_channel,Scheduler::instance().clock()); 
		printf("[SENSING-DBG] Node %d starts sensing on channel %d at time %f \n",
				nodeId_,current_channel,Scheduler::instance().clock()); 
		#endif

#else // LI_MOD

	// We don't sense the receiving channel anymore since it has been done in sense_all_channels()
	// i.e. pu_on_rx is false now, the rest is the same as in senseHandler() 

	pu_on_rx = false;
	ttimer_.start( (transmit_time_ - SWITCHING_DELAY) ); // To keep nodes synchronized - Li
	mac_->checkBackoffTimer(); // Start the backoff timer
#endif 
}

// CRAHNs Model END
// @author:  Marco Di Felice


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
 * senseHandler: handler for sensing timer. Check if PU was detected during the
 * last sensing interval, in case ask the spectrumDecision to switch to a new
 * channel. In case of channel switching, use Spectrum Mobility to perform
 * handoff, and notify the event to the upper layers.
 */

void 
SpectrumManager::senseHandler() {

	// Get current rx channel.
	int current_channel = repository_->get_recv_channel(nodeId_);
	
	#ifdef SENSING_VERBOSE_MODE
	printf("[SENSING-DBG] Node %d is on channel %d and PU activity is %s at time: %f\n", 
			nodeId_, current_channel, (pu_on_rx)?"true":"false", Scheduler::instance().clock());
	#endif

	// Check whether this is a relay node and how many data flows this node is serving.
	int num_prev_relay = repository_->check_rx_set(nodeId_);

	// Output link break info if a relay's rx is interupted by PU.
	if ((num_prev_relay > 0) && pu_on_rx) { 
		for (int i = 0; i < num_prev_relay; i++)
			printf(" [PU Shows Up!] Node: %d Current_Channel: %d Time: %f Set_Times: %d\n", 
					nodeId_, current_channel, CURRENT_TIME, num_prev_relay);
	}

	// Choose a better rx channel after some time.
	if ((num_prev_relay == 1) && (!pu_on_rx)) { 
	// a relay node serving only one data flow
		double randomValue = Random::uniform();
		if (randomValue < 0.25) {
			printf(" [SU Adapts Channel!] Node: %d Current_Channel: %d Time: %f\n", 
					nodeId_, current_channel, CURRENT_TIME);
			pu_on_rx = true; 
			/*
			FILE *fd = fopen("adapt.txt", "a");
			fprintf(fd, "time %f \t node %d\n", CURRENT_TIME, nodeId_);
			fclose(fd);
			*/
		}
	}

	// Check prev-hop tx channels and swith rx channel if rx or pre-hop tx is
	// interrupted by PU.
	if (num_prev_relay != 0) { 

		for (int i = 0; i < MAX_FLOWS; i++) {
			prev_hop[i].id = -1;
			prev_hop[i].flow = -1;
			prev_hop[i].pu_on = false;
		}

		// Get prev-hop node id.
		int node_list[MAX_FLOWS+1];
		int counter_=0;
		node_list[0] = nodeId_;
		for (int i = 0; i < MAX_FLOWS; i++) {
			int flow_ = repository_->read_flow_id(nodeId_, i);
			if(flow_ != -1) {
				prev_hop[counter_].flow = flow_;
				prev_hop[counter_].id = repository_->find_prev_hop(nodeId_, flow_);
				node_list[counter_+1] = prev_hop[counter_].id;
				counter_++;
			}
		}
		if (counter_ != num_prev_relay) { // error check: rx set times
			printf("[!!!WARNING!!!] set num is not correct on node %d\n", nodeId_);
			exit(0);
		}

		// Sense each prev-hop tx channels.
		pu_on_tx = 0;
		int uflow_list[MAX_FLOWS];
		int uflow_list_all[MAX_FLOWS];
		int udst_list[MAX_FLOWS]; 
		int udst_list_all[MAX_FLOWS]; // all flows' dst 

		for (int i = 0; i < num_prev_relay; i++) {
			uflow_list_all[i] = prev_hop[i].flow;
			udst_list_all[i] = repository_->read_flow_dst(prev_hop[i].flow);

			prev_hop[i].pu_on = sensingMod_->sense_prev_tx(prev_hop[i].id, sense_time_, transmit_time_, current_channel);
			if(prev_hop[i].pu_on) {
				uflow_list[pu_on_tx] = prev_hop[i].flow;
				udst_list[pu_on_tx] = repository_->read_flow_dst(prev_hop[i].flow);
				pu_on_tx++;
			}
		}

		// Output link break info if pre-hop tx is interupted by PU.
		if ((!pu_on_rx) && (pu_on_tx > 0)) { 
			for (int i = 0; i < pu_on_tx; i++)
				printf("\n [PU Shows Up!] Node: %d Current_Channel: %d Time: %f Set_Times: %d\n", 
						nodeId_, current_channel, CURRENT_TIME, num_prev_relay);
		}

		// Try to switch rx channel if rx or tx channel is interupted by PU.
		if( pu_on_rx == true || pu_on_tx != 0 ) { 

			// Start handoff timer.
			mobilityMod_->performHandoff();

			// Find the best available channel accroding to metric.
			int next_channel = -1;
			next_channel = repository_->change_channel(node_list, (num_prev_relay+1), CURRENT_TIME);
			if (next_channel == 0) { // error check: use control channel
				printf("[!!!WARNING!!!] node: %d is using control channel.\n", nodeId_);
				exit(0);
			}

			// Condition 1: route break if can't find a channel
			// Condition 2: switch channel if can find a channel
			if (next_channel == -1 && pu_on_rx == true) {
			// no available channel and rx channel is interupted (Condition 1)
				printf(" clean_all_route_channel:\n"); // limark
				for (int i=0; i < num_prev_relay; i++) {
					printf(" ure dst %d\n", udst_list_all[i]);
				}
				// clean all routes
				repository_->clean_route_channel(uflow_list_all, num_prev_relay);
				mac_->notifyUpperLayer(udst_list_all, num_prev_relay);
			} else if ( next_channel == -1 && pu_on_rx == false) {
			// no available channel and only tx channel is interupted (Condition 1)
				printf(" clean_route_channel:\n"); // limark
				printf(" all dst:");
				for(int i=0; i < num_prev_relay; i++) {
					 printf(" %d", udst_list_all[i]);
				}
				printf("\n"); 
				for(int i=0; i < pu_on_tx; i++) {
					printf(" ure dst %d\n", udst_list[i]);
				} 
				// clean impacted routes only
				repository_->clean_route_channel(uflow_list, pu_on_tx);
				mac_->notifyUpperLayer(udst_list, pu_on_tx);
			} else { 
			// channel available (Condition 2)
				printf(" [SU Changes Channel!] Node: %d Current_Channel: %d Next_Channel: %d Time: %f\n\n", 
						nodeId_, current_channel, next_channel, CURRENT_TIME);

				repository_->set_recv_channel(nodeId_, next_channel);
				mac_->load_spectrum(next_channel); // new one - Li
			}

			#ifdef SENSING_VERBOSE_MODE
			printf("[SENSING-DBG] Node %d starts handoff on channel %d to channel %d at time %f \n",
					nodeId_,current_channel,next_channel,Scheduler::instance().clock()); 
			#endif
			
			// Transmit timer will be set after handoff.
			return;

		} // end of if( pu_on_rx == true || pu_on_tx != 0 ) 
	} // end of if(num_prev_relay != 0) 

	// Start transmit timer normally if this is not a relay or if this
	// is a relay but both rx and tx are not interupted by PU.
	pu_on_rx = false;
	ttimer_.start(transmit_time_);
	mac_->checkBackoffTimer();
	sensing_=false;

	#ifdef SENSING_VERBOSE_MODE
	printf("[SENSING-DBG] Node %d starts transmitting on channel %d at time %f \n",
			nodeId_,current_channel,Scheduler::instance().clock()); 
	#endif
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

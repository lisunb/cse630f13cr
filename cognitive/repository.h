// Switchable Interface Implementation START
// @author:  Marco Di Felice	

#ifndef repository_H
#define repository_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random.h>
#include "object.h"


// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< lsun3
#define LI_MOD
// Li's Implementation
#ifdef LI_MOD
// define one spectrum-aware routing metric
// coolest path - highest temp; coolest path - accumulated temp; samer;
//#define CP_AT
//#define CP_HT
#define SAMER
//#define RDM
//#define CRP
// print debug info on screen 
#undef LI_DEBUG
// This number should be the same/larger than the number used in tcl script
#define MAX_NODES	49			/* maximum number of vertices for dijkstra */
#define MAX_NB		MAX_NODES	/* maximum outdegree of a vertex */
#define MAX_FLOWS	5			/* maximum src-dst pairs */
#define MAX_HOP 	26			/* maximun hop counts */
#define MAXD		100.0		/* maximum distance */
#endif // LI_MOD
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	

// Defines the time a node spends on each queue
#define QUEUE_UTILIZATION_INTERVAL 	1.0
// Defines the channel switching delay
#define SWITCHING_DELAY			0.005 // close to switch delay plus hello broadcast delay
// Defines the TIMEOUT_ALIVE to check wheter a node is active on a given channel or not
#define TIMEOUT_ALIVE			2

// Multi-radio multi-channel specification
// Channels/Radio Definition. DO NOT MODIFY HERE!

// Interface Classification
#define CONTROL_RADIO 		0
#define TRANSMITTER_RADIO 	1
#define RECEIVER_RADIO  	2

// Channe/Radio Information 
#define MAX_RADIO	3
#define	MAX_CHANNELS 	11 //#modify to accomodate changes (he says dont modify, why ?)
#define CONTROL_CHANNEL 0

#undef SENSING_VERBOSE_MODE // turn off sensing verbose mode - li

// Channel Entry for receiver nodes
struct repository_entry_recv {
	// receiving channel
	int recv_channel;
	
	#ifdef LI_MOD
	int set; // relay indicator
	int flow[MAX_FLOWS]; // in which flow this node is a relay 
	bool sense_results[MAX_CHANNELS]; // one round sensing results
	#endif
};


// Channel Entry for sender nodes
struct repository_entry_send {
	//Flag indicating wheter the channel is used for transmitting
	bool active;
	//Last time the channel was used
	double time;
};


// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< lsun3
#ifdef LI_MOD

#include "SpectrumData.h"
class SpectrumData;

// Data structures used by Dijkstra Algorithm

// edge
typedef struct {
	int v;							// neighboring vertex
	int channel;					// recv channel of neighboring vertex
	double weight;					// edge weight
} edge;

// graph
typedef struct {
	int nvertices;					// number of vertices in the graph
	int nedges;						// number of edges in the graph
	int degree[MAX_NODES];			// outdegree of each vertex
	edge edges[MAX_NODES][MAX_NB];	// adjacency info
} graph;

// path
typedef struct {
	int is_on;
	int src; // source
	int dst; // destination
	int relay[MAX_HOP];
} repository_entry_path;

// neighbor
typedef struct {
	int node; // node
	bool channel[MAX_CHANNELS];
} repository_neighbor;

#ifdef CRP
// when checking variance, don't count the last off time - the one we are still measuring 
// e.g., 4 = 3(history off time) + 1(the off time being measured)
#define NVS_SAMPLE 11
typedef struct {
	bool is_off[MAX_CHANNELS];
	int	num_off[MAX_CHANNELS];
	double total_off[MAX_CHANNELS];
	double avg_off[MAX_CHANNELS];
	double each_off[MAX_CHANNELS][NVS_SAMPLE];
} nvs_sensing;
#endif // if CRP

#endif // end LI_MOD
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


// Cross-Layer Repository class
class Repository : public NsObject {

        public:

		 // Initializer
		 Repository();
		 int command(int argc, const char*const* argv);
		 void recv(Packet*, Handler*);
	 		 
		 // Set/Get Function for the Receiver Channel Table
		 int get_recv_channel(int node);
		 void set_recv_channel(int node, int channel);
		
		 // Set/Get Function for the Sender Channel Table
		 void update_send_channel(int node, int channel, double time);
		 bool is_channel_used_for_sending(int node, int channel, double timeNow);

#ifdef LI_MOD
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		/*******************
		 * initialization 
		 *******************/

		bool is_sd_pointer_set();
		void set_sd_pointer(SpectrumData *dataMod_); // set spectrum data pointer  
		void update_nb(int node, int nb); // update neighbor table
		void update_nb(int node, int neighbor, int chan); // update neighbor table
		void print_nb(int node); // debug - print neighbor and channel info

		/********************************
		 * channel sensing and switching 
		 ********************************/
		/*
		bool repository_in_sensing[MAX_NODES]; // in sensing indicator
		void mark_in_sensing(int node); // mark in sensing
		void clean_in_sensing(int node); // clean in sensing
		*/
		int check_rx_set(int id); // how many flows it serve
		void mark_channel(int node, int channel, bool appear); // mark channel in one sensing period
		#ifdef CRP
		void update_nvs_table(int nodeId, int channelId, bool puOff); 
		void update_average_channel_utility();
		bool check_channel_average(int node, int channel, double time);
		bool check_channel_variance(int node, int channel, double time);
		#endif
		void update_active_count(int node, int channel); // update pu showing up times
		void update_channel_utility(int node, int counter); // update channel utility
		double get_channel_utility(int node, int channel); // show PUs' showup ration on each channel
		int change_channel(int *list, int node_num, double time); // change channel by metric

		/********************
		 * route management 
		 ********************/

		// look for the best path and assign channels
		int set_route_channel(int src, int dst, double time); // return 1 if done
		int read_flow_id(int node, int index);
		int read_flow_dst(int index);
		int find_prev_hop(int node, int flow);

		/****************
		 * AODV support 
		 ****************/

		int get_path_id(int dst, int src); // find path id for aodv sendReply()
		int get_relay_by_hop(int id, int count);
		int check_path_state(int id);
		void clean_route_channel(int *flow_list, int flow_num);
		void clean_all_route_channel(int node);

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#endif // LI_MOD
	
	private:

#ifdef LI_MOD // LI_MOD

		bool is_common_channel(int channel, int *node, int num); // check channel for a list of nodes
		// dijkstra algorithm related
		int construct_graph(graph *g, double time); // construct graph for dijkstra
		double cal_link_wt(int host, int nb, int channel, double time); // calculate link metric 
		void cal_min_wt_link(graph *g, int node, int neighbor, double time); // calculate minimum weight link/channel between pairs 
		void dijkstra(graph *g, int start, int parent[]); // dijkstra by Steven S. Skiena
		int record_path(graph *g, int start, int end, int parent[]); // record dijkstra result in repository

		// repositories
		repository_entry_recv repository_table_rx[MAX_NODES]; // contains the channels used for receiving by node i 
		repository_entry_send repository_table_tx[MAX_NODES][MAX_CHANNELS]; // contains the information (active/time) for sending node i and channel j
		repository_entry_path repository_table_path[MAX_FLOWS]; // all routing routes 
		repository_neighbor repository_table_nb[MAX_NODES][MAX_NB];

		int repository_node_nb[MAX_NODES][MAX_NB]; // neighbor table
		int repository_active_count[MAX_NODES][MAX_CHANNELS]; // times of pu show-up
		double repository_channel_utility[MAX_NODES][MAX_CHANNELS]; // channel utilities
		#ifdef CRP
		double average_channel_utility[MAX_CHANNELS];
		nvs_sensing nvs_table[MAX_NODES];	
		#endif

		bool sd_pointer_set_; // indicator of setting spectrum data pointer
		SpectrumData *sd_; 

#else // no LI_MOD
		// Receiver Channel table: repository_table[i] contains the channels used for receiving by node i
		repository_entry_recv repository_table[MAX_NODES];
		// Sender Channel table: repository_table_sender[i][j] contains the information (active/time) for sending node i and channel j
		repository_entry_send repository_table_sender[MAX_NODES][MAX_CHANNELS];
#endif
		// Returns a random channel between 1 and MAX_CHANNELS
		int get_random_channel();
};

// Switchable Interface Implementation END

#endif

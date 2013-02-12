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
// Li's Implementation
#define LI_MOD

#ifdef LI_MOD

// print debug info on screen 
#undef LI_DEBUG

// coolest path - highest temp; coolest path - accumulated temp; samer;
#define CP_AT
//#define CP_HT
//#define SAMER
//#define RDM
//#define CRP

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
#define SWITCHING_DELAY			0.001
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


#ifndef LI_MOD
#define SENSING_VERBOSE_MODE
#endif //No LI_MOD


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
// -- edge, graph
typedef struct {
	int v;				/* neighboring vertex */
	double weight;			/* edge weight */
	int channel;			/* recv channel of neighboring vertex */
} edge;

typedef struct {
	edge edges[MAX_NODES][MAX_NB];	/* adjacency info */
	int degree[MAX_NODES];		/* outdegree of each vertex */
	int nvertices;			/* number of vertices in the graph */
	int nedges;			/* number of edges in the graph */
} graph;

// Data structure used to record path and channel allocation results  
typedef struct {
	int is_on;
	int src; // source
	int dst; // destination
	int relay[MAX_HOP];
} repository_path;

#ifdef CRP
#define NV_PERIOD 50
typedef struct {
	int sensing[NV_PERIOD][MAX_CHANNELS];
} nvs_sensing;
#endif // end CRP

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

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#ifdef LI_MOD

		/*******************
		 * Initialization 
		 *******************/

		// getting spectrum data pointer indicator
		int sd_pointer_set;
		// set spectrum data pointer 
		void set_sd_pointer(SpectrumData *dataMod_);

		// update neighbor table	
		void update_nb(int node, int nb);

		/****************************
		 * Channel sensing related
		 ****************************/

		// in sensing period indicator
		bool repository_in_sensing[MAX_NODES];
		// mark in sensing
		void mark_in_sensing(int node);
		// clear in sensing
		void clear_in_sensing(int node);
		// check whether a node is a relay node in channel sensing period
		int check_recv_set(int id);
		// One node's observation for a channel in one sensing period 
		void mark_channel(int node, int channel, bool appear);
		#ifdef CRP
		void update_nvs_table(int node, int counter, int channel, bool appear); 
		bool check_variance(int node, int channel, double time);
		#endif
		// update how many times one channel is used by a pu in SpectrumSensing
		void update_sensing_result(int node, int channel);
		// update all channels' utilities in SpectrumSensing
		void update_channel_u(int node, int counter);
		// change a channel according to a metric
		int change_channel(int *list, int node_num, double time);
		bool channel_for_all(int channel, int *node, int num);

		int read_flow_id(int node, int index);
		int read_flow_dst(int index);
		int find_prev_hop(int node, int flow);

		/*******************************
		 * Dijkstra algorithm related
		 *******************************/

		// look for the best path and assign channels
		int set_route_channel(int src, int dst, double time); // return 1 if done
		// read graph for dijkstra
		int construct_graph(graph *g, double time);
		// calculate minimal weight with each channel according to a metric
		void cal_min_wt_link(graph *g, int node, int neighbor, double time);
		// calculate the link metric value according to a metric
		double cal_link_wt(int host, int nb, int channel, double time); 
		// dijkstra by Steven S. Skiena
		void dijkstra(graph *g, int start, int parent[]);
		// write dijkstra's result in repository
		int record_path(graph *g, int start, int end, int parent[]);

		/****************
		 * AODV support 
		 ****************/

		// find path id for aodv sendReply()
		int get_path_id(int dst, int src);
		int get_addr_by_hop(int id, int count);
		int check_path_state(int id);
		void clean_route_channel(int *flow_list, int flow_num);
		void clean_all_route_channel(int node);

		#endif
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	private:

		// Receiver Channel table: repository_table[i] contains the channels used for receiving by node i
		repository_entry_recv repository_table[MAX_NODES];
		// Sender Channel table: repository_table_sender[i][j] contains the information (active/time) for sending node i and channel j
		repository_entry_send repository_table_sender[MAX_NODES][MAX_CHANNELS];
		// Returns a random channel between 1 and MAX_CHANNELS
		int get_random_channel();

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#ifdef LI_MOD
		// global neighbor table
		int repository_node_nb[MAX_NODES][MAX_NB];
		// store how many times one channel is used by a pu
		int repository_table_sensing[MAX_NODES][MAX_CHANNELS];
		// store all channels' utilities
		double repository_channel_u[MAX_NODES][MAX_CHANNELS];
		#ifdef CRP
		nvs_sensing nvs_table[MAX_NODES];	
		#endif // end LI_MOD 

		// store all paths for RREP
		repository_path repository_table_path[MAX_FLOWS];

		// pointer to spectrum data 
		SpectrumData *sd_; 
		#endif // end LI_MOD
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>	
};

// Switchable Interface Implementation END

#endif



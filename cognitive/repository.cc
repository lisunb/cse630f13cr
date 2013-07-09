// Switchable Interface Implementation START
//
// @author:  Marco Di Felice	
//
// Class Repository 
// Cross-Layer Repository to enable channel information sharing between MAC and routing protocols

#include "repository.h"

static class Repositoryclass : public TclClass {
public:
        Repositoryclass() : TclClass("CrossLayerRepository") {}
        TclObject* create(int argc, const char*const* argv) {
          return (new Repository());
        }
} class_repository;


// Initializer
Repository::Repository() {

#ifndef LI_MOD // no LI_MOD
	// Set randomly the receiver channel for each node	
	for (int i=0; i<MAX_NODES; i++) {
		int channel=get_random_channel();
		repository_table_rx[i].recv_channel= channel;
	}

	// Initialize each sending channel as NOT active for each node
	for (int node=0; node<MAX_NODES; node++) 
		for (int channel=0; channel< MAX_CHANNELS; channel++) 
			repository_table_tx[node][channel].active=false;

#else // LI_MOD
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// Intialize node table - node i, channel j, path k 
	for (int i = 0; i < MAX_NODES; i++) {
		// set the rx channel randomly for each node
		repository_table_rx[i].recv_channel = get_random_channel();
		repository_table_rx[i].set = 0;

		for(int j = 0; j < MAX_CHANNELS; j++) {
			// rx and tx on node i, channel j 
			repository_table_rx[i].sense_results[j] = false;
			repository_table_tx[i][j].active = false;
			// active counts and utilites on node i / rx i
			repository_active_count[i][j] = 0;
			repository_channel_utility[i][j] = 0.0;
		}		

		// avoid using control channel
		repository_channel_utility[i][0] = 1.0;

		// flow tables on node i
		for(int k = 0; k < MAX_FLOWS; k++)
			repository_table_rx[i].flow[k] = -1;
	}

	// Initialize neighbor tables - node i, neighbor j
	for(int i = 0; i < MAX_NODES; i++) { 
		for(int j = 0; j < MAX_NB; j++)
			repository_node_nb[i][j] = -1;
	}

	for(int i = 0; i < MAX_NODES; i++) {
		for(int j = 0; j < MAX_NB; j++) {
			repository_table_nb[i][j].node = -1;
			for(int k = 0; k < MAX_CHANNELS; k++)
				repository_table_nb[i][j].channel[k] = false;
		}
	}

	/*
	// Initialize sensing indicators
	for(int i = 0; i < MAX_NODES; i++) { 
		repository_in_sensing[MAX_NODES] = false;
	}
	*/

	#ifdef CRP
	for (int i = 0; i < MAX_NODES; i++) { 
		for(int j = 0; j < MAX_CHANNELS; j++) {
			nvs_table[i].num_off[j] = 0;
			nvs_table[i].is_off[j] = false;
			nvs_table[i].avg_off[j] = 0.0;
			nvs_table[i].total_off[j] = 0.0;
			for (int k = 0; k < NVS_SAMPLE; k++)
				nvs_table[i].each_off[j][k] = 0.0;
		}
	}

	for (int i = 0; i < MAX_CHANNELS; i++) {
		average_channel_utility[i] = 0.0;
	}

	// initialize channel weights
	double max_dist = 250.0;
	channel_wt[0] = max_dist / max_dist;
	for (int i = 1; i < MAX_CHANNELS; i++) {
		int t = (i + 1) / 2;
		channel_wt[i] = (max_dist - (double)(t-1) * 40.0) / max_dist;
	}

	#endif // end CRP

	// Initialize route tables, path i, hop j
	for(int i = 0; i < MAX_FLOWS; i++) {
		repository_table_path[i].src = -1;
		repository_table_path[i].dst = -1;
		repository_table_path[i].is_on = 0;
		for(int j = 0; j < MAX_HOP; j++) {
			repository_table_path[i].relay[j] = -1;
		}
	}

	// Indicator for getting SpectrumData pointer
	sd_pointer_set_ = false;

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#endif //LI_MOD
}


//get_recv_channel: Return the receiving channel for a node
int 
Repository::get_recv_channel(int node) {
	if (node < MAX_NODES)
		return repository_table_rx[node].recv_channel;
	else
		return -1;
}
	 

//set_recv_channel: Set the receiving channel for a node
void 
Repository::set_recv_channel(int node, int channel) {
	if (node < MAX_NODES)
		repository_table_rx[node].recv_channel=channel;

}

		
// update_send_channel: Set the sending channel as active, at the current time
void 
Repository::update_send_channel(int node, int channel, double time) {

	if (node < MAX_NODES)  {
		
		repository_table_tx[node][channel].active=true;
		repository_table_tx[node][channel].time=time;
	
	 }

}
		 

//is_channel_used_for_sending: Check whether a given sending channel is active for a given node
bool 
Repository::is_channel_used_for_sending(int node, int channel, double timeNow) {

	if (repository_table_tx[node][channel].active) {
		if (timeNow - repository_table_tx[node][channel].time > TIMEOUT_ALIVE)
			repository_table_tx[node][channel].active=false;
	}
	
	return repository_table_tx[node][channel].active;
	
}


//get_random_channel: Return a random channel between 1 and MAX_CHANNELS
int 
Repository::get_random_channel() {
	
	int channel=((int)(Random::uniform()*MAX_CHANNELS))+1;		
	if (channel >= MAX_CHANNELS)
		channel = MAX_CHANNELS-1;
	return channel;
}


// recv: Empty method
void
Repository::recv(Packet*, Handler* = 0) {

}

// command: Empty method
int
Repository::command(int argc, const char*const* argv) {
}

#ifdef LI_MOD
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/****************************
 * Set spectrum data pointer 
 ****************************/

void 
Repository::set_sd_pointer(SpectrumData *dataMod_ ) {

        if(sd_pointer_set_)
			return;

		sd_ = dataMod_;
		sd_pointer_set_ = true;

		#ifdef LI_DEBUG
		printf("[Set Spectrum Data Pointer in Repository] Successfully!\n");
		#endif
}

bool
Repository::is_sd_pointer_set() {
	return sd_pointer_set_;
}
 
/********************************
 * Update global neighbor table 
 ********************************/

void
Repository::update_nb(int node, int nb) {

	int i, is_nb = 0;

	//check if this nb exists
	for(i = 0; i < MAX_NB; i++) {
		if(repository_node_nb[node][i] == -1)
			break; // i is the position for this nb;
		if(repository_node_nb[node][i] == nb) {
			is_nb = 1;
			break;
		}
	}

	if((is_nb == 0)&&(i != MAX_NB)) 
		repository_node_nb[node][i] = nb;
}

void
Repository::update_nb(int node, int neighbor, int chan) {

	int nb, is_set = 0; // nb is index

	// check if this nb exists
	for(nb = 0; nb < MAX_NB; nb++) {
		if (repository_table_nb[node][nb].node == neighbor) {
			if (repository_table_nb[node][nb].channel[chan])
				is_set = 1;
			break;
		}
		if(repository_table_nb[node][nb].node == -1)
			break;
	}

	// remember this neighbor and channel
	if((is_set == 0) && (nb != MAX_NB)) { 
		repository_table_nb[node][nb].node = neighbor;
		repository_table_nb[node][nb].channel[chan] = true;
	}
}

void
Repository::print_nb(int node) {

	printf("Neighbor table of node %d: ", node);
	for(int nb = 0; nb < MAX_NB; nb++) {
		if(repository_table_nb[node][nb].node != -1)
			printf("%d ", repository_table_nb[node][nb].node);
	}
	printf("\n");

	for(int ch = 0; ch < MAX_CHANNELS; ch++) {
		printf("On channel %d: ", ch);
		for(int nb = 0; nb < MAX_NB; nb++) {
			if((repository_table_nb[node][nb].node != -1) && repository_table_nb[node][nb].channel[ch])
				printf("%d ", repository_table_nb[node][nb].node);
		}
		printf("\n");
	}
}

/**********************************************
 * Functions used in channel sensing
 **********************************************/

/*
// mark in sensing
void
Repository::mark_in_sensing(int node) {
	
	repository_in_sensing[node]=true;
}

// clear in sensing
void
Repository::clean_in_sensing(int node) {

	repository_in_sensing[node]=false;
}
*/

// check wehther a node is a relay node
int
Repository::check_rx_set(int node) {
	return repository_table_rx[node].set;
}

// Record one node's observation for a channel 
// - set it false if finding PU using it
void
Repository::mark_channel(int node, int channel, bool appear) {

	repository_table_rx[node].sense_results[channel] = appear;

}

#ifdef CRP
void
Repository::update_nvs_table(int nodeId, int channelId, bool puOff) {

	int i = nodeId;
	int j = channelId;

	if (nvs_table[i].is_off[j] == false && puOff == true) { // start a new off sample
		// mark the channel as off
		nvs_table[i].is_off[j] = true;
		// update off counter and total off time
		nvs_table[i].num_off[j]++;
		nvs_table[i].total_off[j] += 1.0;
		// clean and update nvs sample in the next position
		int nextNvsId = (nvs_table[i].num_off[j])%NVS_SAMPLE;
		nvs_table[i].each_off[j][nextNvsId] = 1.0;
	}
	else if (nvs_table[i].is_off[j] == true && puOff == true) { // continue a off sample
		// update total off time
		nvs_table[i].total_off[j]+=1.0;
		// update nvs sample in current position
		int currentNvsId = (nvs_table[i].num_off[j])%NVS_SAMPLE;
		nvs_table[i].each_off[j][currentNvsId] += 1.0;
	}
	else if (nvs_table[i].is_off[j] == true && puOff == false) { // finish a off sample
		// mark the channel as on
		nvs_table[i].is_off[j] = false;
		// update avg off time
		nvs_table[i].avg_off[j] = nvs_table[i].total_off[j] / nvs_table[i].num_off[j];
		/*
		if (i == 0 && j == 1) {
			FILE *fd = fopen("puoff.txt", "a");
			int currentNvsId = (nvs_table[i].num_off[j])%NVS_SAMPLE;
			fprintf(fd, "%f %f\n",nvs_table[i].each_off[j][currentNvsId], nvs_table[i].avg_off[j]);
			fclose(fd);
		}
		*/
	}
	else { // equals: nvs_table[i].is_off[j] == false && puOff == false
		// node is waiting for next sample; do nothing
	}
}

void
Repository::update_average_channel_utility() {

	for (int i = 1; i < MAX_CHANNELS; i++) {
		average_channel_utility[i] = 0.0;
		for (int j = 0; j < MAX_NODES; j++) {
			average_channel_utility[i] += repository_channel_utility[j][i];
		}
		average_channel_utility[i] /= MAX_NODES;
	}
}

bool
Repository::check_channel_average(int node, int channel) {
	// return false if larger than average 
	/*
	FILE *fd = fopen("avg.txt", "a");
	fprintf(fd, "[t] %f \t[n] %d \t[c] %d \t[avg] %f \t [u] %f\n",
			time, node, channel, average_channel_utility[channel], repository_channel_utility[node][channel]);
	fclose(fd);
	*/
	if (repository_channel_utility[node][channel] > average_channel_utility[channel])
		return false;
	else
		return true;
}

bool
Repository::check_channel_variance(int node, int channel) {
	// return false if larger than threshold
	//double var_threshold = 10000.0;
	double var_threshold = 0.5*pow(nvs_table[node].avg_off[channel], 2);
	double variance = 0.0;
	int num_sample = NVS_SAMPLE - 1;
	// get start Id
	int sampleId;
	if (nvs_table[node].is_off[channel] == false) { // check nvs-1 samples containing current position
		sampleId = (nvs_table[node].num_off[channel] + 2) % NVS_SAMPLE;
	}
	else { // check nvs-1 samples NOT from current position
		sampleId = (nvs_table[node].num_off[channel] + 1) % NVS_SAMPLE;
	}
	// calculate variance
	for (int i = 0; i < num_sample; i++) {
		if (nvs_table[node].each_off[channel][sampleId] > nvs_table[node].avg_off[channel]) {
			// do nothing
		}
		else {
			variance += pow((nvs_table[node].avg_off[channel] - nvs_table[node].each_off[channel][sampleId]), 2);
		}
		sampleId = (sampleId + 1) % NVS_SAMPLE;
	}
	variance /= num_sample;
	// check with threshold
	/*
	FILE *fd = fopen("var.txt", "a");
	fprintf(fd, "[t] %f \t[n] %d \t[c] %d \t[var] %f\n",
			time, node, channel, variance);
	fclose(fd);
	*/
	if (variance > var_threshold)
		return false;
	else
		return true;
}

// check whether channels satisfies crp constraints
void
Repository::check_channel_st(bool *st_list, int node) {

	st_list[0] = false;

	for (int ch = 1; ch < MAX_CHANNELS; ch++) {
		if ((check_channel_average(node, ch) == false) ||
			(check_channel_variance(node, ch) == false)) {
		// check average and variance
			st_list[ch] = false;
		} else if (ch % 2 == 0) { 
		// the last channel in a spectrum band
			if (repository_channel_utility[node][ch] < repository_channel_utility[node][(ch-1)]) {
					st_list[ch] = true;
					st_list[(ch-1)] = false;
			} else {
				st_list[ch] = false;
			}
		} else {
		// the other channels in a spectrum band
			st_list[ch] = true;
		}
	}
}
#endif // if CRP

// Update how many times one channel is found being used by a PU 
void
Repository::update_active_count(int node, int channel) {
	repository_active_count[node][channel]++;
}

// Update channel' utilities after channel sensing
void
Repository::update_channel_utility(int node, int counter) {

	for(int i = 1; i < MAX_CHANNELS; i++)
		repository_channel_utility[node][i] = (double)repository_active_count[node][i]/(double)counter;
}

// show PUs' show up ratio on each channel
double
Repository::get_channel_utility(int node, int channel) {
	return repository_channel_utility[node][channel];
}

/********************************************************
 * Functions used for joint path and channel allocation
 ********************************************************/

// calulate the link metric value accroding to a metric
double
Repository::cal_link_wt(int host, int nb, int channel, double time) {

	double current_time = time;
	double metric_value_ = 0.0;

#ifdef CP_AT
	metric_value_ = 1 - (1 - repository_channel_utility[host][channel])*(1 - repository_channel_utility[nb][channel]);
#endif // end if CP_AT
	
#ifdef CP_HT
	metric_value_ = 1 - (1 - repository_channel_utility[host][channel])*(1 - repository_channel_utility[nb][channel]);
#endif // end if CP_HT

#ifdef SAMER
	// searching order is important since we care about transmission interference
	// count how mamy NBs are using this channel in 2 hops
	int nb_counter = 0; // number of 1-hop and 2-hop nb
	int nb_list[MAX_NB]; // 2-hop nb list
	for (int i = 0; i < MAX_NB; i++)
		nb_list[i] = -1;

	// search 1-hop nb
	for (int i = 0; i < MAX_NB; i++) {
		if (repository_table_nb[host][i].node == -1)
			break;
		if (repository_table_nb[host][i].channel[channel]) {
			nb_list[i] = repository_table_nb[host][i].node;
			nb_counter++;
		}
	}

	// search 2-hop nb
	int nb1_num = nb_counter;
	for (int i = 0; i < nb1_num; i++) {
		int nb1 = nb_list[i]; // 1-hop nb
		for (int j = 0; j < MAX_NB; j++) {
			if (repository_table_nb[nb1][j].node == -1)
				break;
			if (repository_table_nb[nb1][j].channel[channel]) {
				int nb2 = repository_table_nb[nb1][j].node; // 2-hop nb
				bool appear = false;
				for (int k = 0; k < nb_counter; k++) { // whether in nb_list
					if (nb_list[k] == nb2)
						appear = true;
				}
				if (appear == false && nb2 != host) {
					nb_list[nb_counter] = nb2;
					nb_counter++;
				}
				if (nb_counter == MAX_NB) {
					printf("[!!!WARNING!!!] 2-hop nb num exceed upper limit.\n");
					exit(0);
				}
			}
		}
	}

	// check 2-hop tx interference
	int channel_count = 1;
	for(int i = 0; i < nb_counter; i++) {
		if(is_channel_used_for_sending(nb_list[i], channel, current_time))
			channel_count++;
	}

	double av_host = 1.0 - repository_channel_utility[host][channel];
	double av_nb = 1.0 - repository_channel_utility[nb][channel];
	double av_min = av_host>av_nb ? av_nb:av_host;
	metric_value_ = 1.0 - av_min*(1.0 - sd_->spectrum_table_[channel].per[host][nb])/(double)channel_count;
#endif // end if SAMER
 
#ifdef CRP
	metric_value_ = 1.0 - channel_wt[channel];
#endif // end if CRP

	return metric_value_;
}

// calculate link weights between a neighboring pair
// and find out the best channel
void
Repository::check_neighbor(graph *g, int node, int neighbor, double time) {

	// variable initialization
	int nb = g->edges[node][neighbor].v; // real neighbor id
	int channel_ = -1; // current best channel
#ifdef SAMER
	double weight_ = 0.0; // cumulative weight
	double min_w = MAXD; // current minimum weight
#else
	double weight_ = MAXD; // current minimum weight
#endif

#ifdef CRP // check if a channel satisfies crp constraints
	bool max_link_wt = 100.0;
	bool channel_st[MAX_CHANNELS];
	check_channel_st(channel_st, node);
#endif

	// check all channels one by one
	if (repository_table_rx[nb].set == 0) {
	// rx channel is not set
		for (int ch = 1; ch < MAX_CHANNELS; ch++) {
			if (repository_table_nb[node][neighbor].channel[ch]) {
			// hold a neighbor relationship on channel "ch"
				double t_ = cal_link_wt(node, nb, ch, time);
#ifdef CRP // CRP
				if (!channel_st[ch]) {
					t_ = max_link_wt;
				}
#endif

#ifndef SAMER
				if (t_ < weight_) {
					weight_ = t_;
					channel_ = ch;
				}
#else // SAMER
				weight_ += t_;
				if (t_ < min_w) {
					min_w = t_;
					channel_ = ch;
				}
#endif
			}
		}
	} else {
	// rx channel is already set
		channel_ = repository_table_rx[nb].recv_channel;
#ifndef SAMER
		weight_ = cal_link_wt(node, nb, channel_, time);
	#ifdef CRP
		if (!channel_st[channel_]) {
			weight_ = max_link_wt;
		}
	#endif
#else // SAMER
		int route_count = repository_table_rx[nb].set + 1;
		for (int ch = 1; ch < MAX_CHANNELS; ch++) {
			if (repository_table_nb[node][neighbor].channel[ch]) {
			// hold a neighbor relationship on channel "ch"
				double t_ = cal_link_wt(node, nb, ch, time);
				weight_ = weight_ + 1 - (1 - t_)/(double)route_count; // shared by route
			}
		}
#endif
	}

	// store the best weight and channel of for this neighbor	
	g->edges[node][neighbor].weight = weight_;
	g->edges[node][neighbor].channel = channel_;
}

// construct graph for dijkstra
// initialize graph and insert edges with min weight link/channel
void
Repository::construct_graph(graph *g, double time) {

	double current_time = time;

	// initialize graph
	g->nvertices = 0;
	g->nedges = 0;

	for (int i = 0; i < MAX_NODES; i++) {  
		g->degree[i] = 0;
		for (int j = 0; j < MAX_NB; j++) {
			g->edges[i][j].v = -1;
			g->edges[i][j].weight = MAXD;
			g->edges[i][j].channel = -1;
		}	
	}

	// check edges
	g->nvertices = MAX_NODES; // assume MAX_NODES is the number of all nodes used

	for (int i = 0; i < g->nvertices; i++) {
		for (int j = 0; j < MAX_NB; j++) {
			if (repository_table_nb[i][j].node == -1)
			// the end of neighbor list
				break;
			if (g->degree[i] == (MAX_NB-1)) {
			// check the number of neighbor
				printf("\n[!!!WARNING!!!] Node %d will exceed max degree.\n",i);
				exit(0);
			}
			
			// check whether a valid neighbor
			int nb_id = repository_table_nb[i][j].node; // real neighbor id
			int nb_rx_set = repository_table_rx[nb_id].set;
			int nb_rx_ch = repository_table_rx[nb_id].recv_channel;

			if (nb_rx_ch <= 0) {
			// check correctness of rx channel
				printf("\n[!!!WARNING!!!] Node %d is using wrong rx channel.\n",i);
				exit(0);
			}

			if ((nb_rx_set == 0) ||
				((nb_rx_set > 0) && repository_table_nb[i][j].channel[nb_rx_ch])) {
			// neighbor rx is not set or the neighbor is a neighbor on its rx channel
				g->edges[i][j].v = nb_id; // insert node
				check_neighbor(g, i, j, current_time); // check channels on this neighbor
				g->degree[i] ++;
				g->nedges ++;		
			}
		}
	}
}

void 
Repository::dijkstra(graph *g, int start, int parent[]) {

	bool intree[MAX_NODES];		/* is the vertex in the tree yet? */
	double distance[MAX_NODES];	/* distance vertex is from start */

	for (int i = 0; i < g->nvertices; i++) {
		intree[i] = false;
		distance[i] = MAXD;
		parent[i] = -1;
	}

	// set searching start point (from source node)
	int cur_node = start; 
	distance[cur_node] = 0.0;

	// start searching procedure
	while (intree[cur_node] == false) {
		// delete current node from candidate set
		intree[cur_node] = true;
		// check all of this current node's links
		for (int i = 0; i < g->degree[cur_node]; i++) {
			int nb_node = g->edges[cur_node][i].v; // neighbor node (candidate next vertex)
			double weight = g->edges[cur_node][i].weight; // edge weight (edge length)
			
			#ifdef CP_AT
			if (distance[nb_node] > (distance[cur_node]+weight)) {
				distance[nb_node] = distance[cur_node]+weight;
				parent[nb_node] = cur_node;
			}
			#endif 

			#ifdef CP_HT
			double max_t = (distance[cur_node]>weight?distance[cur_node]:weight);

			if (distance[nb_node] > max_t) {
				distance[nb_node] = max_t;
				parent[nb_node] = cur_node;
			}
			#endif

			#ifdef SAMER 
			double max_t = (distance[cur_node]>weight?distance[cur_node]:weight);

			if (distance[nb_node] > max_t) {
				distance[nb_node] = max_t;
				parent[nb_node] = cur_node;
			}
			#endif

			#ifdef CRP
			if (distance[nb_node] > (distance[cur_node]+weight)) {
				distance[nb_node] = distance[cur_node]+weight;
				parent[nb_node] = cur_node;
			}
			#endif 
		}
		// look for the next "current node"
		double shortest_dist = MAXD;
		for (int i = 0; i < g->nvertices; i++) {
			if ((intree[i] == false) && (distance[i]) < shortest_dist) {
				shortest_dist = distance[i];
				cur_node = i;
			}
		}
	}
}

// record dijkstra result in repository
int
Repository::record_path(graph *g, int start, int end, int parent[]) {

	// find one entry position in global route table	
	int i; // i is the entry point
	for (i = 0; i < MAX_FLOWS; i++) {
		if (repository_table_path[i].is_on == 0) {
			break;
		}
		if (i == (MAX_FLOWS-1)) {
			printf("\n[!!!WARNING!!!] The number of flows exceeds the upper limit.\n");
			exit(0);
		}
	}

	// set src, dst and state
	repository_table_path[i].is_on = 1;
	repository_table_path[i].src = start;
	repository_table_path[i].dst = end;	

	// record nodes for a path in reverse order, i.e., from dst to src
	// path contains dst and src 
	int j = 0;
	repository_table_path[i].relay[j] = end; // record dst

	int next_ = end;
	while (next_ != start) { // record the other nodes including src
		j++;
		if (j == MAX_HOP) {
 			printf("\n[!!!WARNING!!!] hop counts betw %d and %d exceeds upper limit!\n", 
					start, end);
			exit(0);
		}
		repository_table_path[i].relay[j] = parent[next_];
		next_ = parent[next_];
	}

	// record rx channels and serving flows for all nodes
	// don't need to src's rx channel and serving flow 
	next_ = end;
	while (next_ != start) {

		// find entry point in parent table
		int parent_ = parent[next_];
		int k;
		for (k = 0; k < g->degree[parent_]; k++) {
			if (g->edges[parent_][k].v == next_)
				break;
		}

		// set rx channels 
		if (repository_table_rx[next_].set == 0) {
		// node chosen as a relay for the first time
			repository_table_rx[next_].recv_channel = g->edges[parent_][k].channel;
			repository_table_rx[next_].set++;
		} else {
		// an intersecting node of multiple flows
			repository_table_rx[next_].set++;
			// error check: inconsistent rx channel	
			if (repository_table_rx[next_].recv_channel != g->edges[parent_][k].channel) {
				printf("\n[!!!WARNING!!!] The intersecting node %d is set to use another channel.\n", 
						next_);
				exit(0);
			}
		}

		// set flows this node serve
		// error check: duplicated entries
		for (int f = 0; f < MAX_FLOWS; f++) {
			if (repository_table_rx[next_].flow[f] == i) { // i is the entry point
				printf("\n[!!!WARNING!!!] Node: %d shows up twice in route: %d.\n", next_, i);
				exit(0);
			}
		}
		// find local flow entry point
		for (int f = 0; f < MAX_FLOWS; f++) {
			if (repository_table_rx[next_].flow[f] == -1) {
				repository_table_rx[next_].flow[f] = i;
				break;
			}
		}

		next_ = parent[next_];		
	}

	return i; // return entry point
}	

// route and channel joint allocation
int
Repository::set_route_channel(int src, int dst, double time) {

#ifdef CRP
	update_average_channel_utility();
#endif

	// Step 1 - Check whether we have set route and channel for this src-dst.
	for(int i = 0; i < MAX_FLOWS; i++) {
		if( repository_table_path[i].is_on == 1 &&
			repository_table_path[i].dst == dst &&
		    repository_table_path[i].src == src ) { // Check whether path exists
				return 0;
		}
	}

	// Step 2 - Read/Contruct graph with repository.
	graph g;
	construct_graph(&g, time); 

	// Step 3 - Search the best path.
	int parent[MAX_NODES]; // discovery relation
	dijkstra(&g, src, parent);

	// Step 4 - Record the path & channel allocation results in repository.
	int entry_point = record_path(&g, src, dst, parent);

	// print route information
	int hop_count; 
	for (hop_count = 0; hop_count < MAX_HOP; hop_count++) {
		if(repository_table_path[entry_point].relay[hop_count] == -1)
			break;
	}
	printf("[ROUTE INFO] Src: %d Dst: %d Time: %f Hop: %d Relay:", src, dst, time, (hop_count -1));
	for (int i = (hop_count -1); i >= 0; i--)
		printf(" %d", repository_table_path[entry_point].relay[i]);
	printf(" Channel:");
	for (int i = (hop_count -1); i >= 0; i--)
		printf(" %d", repository_table_rx[(repository_table_path[entry_point].relay[i])].recv_channel);
	printf("\n");

	#ifdef LI_DEBUG
	printf("Print the Number of PU Appearance on Each Channel.\n");
	for (int i = 0; i < MAX_NODES; i++) {
		printf("Observation of Node %d:\n", i);
		for (int j = 1; j < MAX_CHANNELS; j++) {
			printf("%d ", repository_active_count[i][j]);
		}
		printf("\n");
	}
	#endif

	return 0;
}

/****************************************************
 * Functions for channel switching and route repair
 ****************************************************/

// Check whether an available channel for all nodes
bool
Repository::is_common_channel(int channel, int *node, int num) {

	bool available=true;
	int node_;

	for(int i=0; i < num; i++) {
		node_ = node[i];
		if(repository_table_rx[node_].sense_results[channel] == false) {
			available=false;
			break;
		}
	}

	return available;
}

// Change rx channel when finding a PU using it
int
Repository::change_channel(int *list, int node_num, double time) {

#ifdef CRP
	update_average_channel_utility();
#endif // if CRP

	double current_time = time;
	int node_list[MAX_FLOWS+1]; // 0 is for RX

	// limark
	printf("Number of nodes: %d\n", node_num);
	printf("Node lists: ");
	if(node_num > MAX_FLOWS + 1) {
		printf("[!!!WARNING!!!] node_num exceeds MAX_FLOW + 1 in change_channel.\n");
		exit(0);
	}

	for(int i=0; i < node_num; i++) {
		node_list[i] = list[i];
		// limark
		printf("%d ", node_list[i]);
	}
	
	// limark
	printf("\n");

	int host_=node_list[0];

	// Error Check
	if( repository_table_rx[host_].set != node_num - 1 ) {
		printf("\n[!!!WARNING!!!] node %d set: %d node_num: %d not correct while changing channel.\n",
				node_list[0], repository_table_rx[host_].set, node_num);
		exit(0);
	}

	// Check available channels we can use 
	int channel_num = 0;
	int channel_list[MAX_CHANNELS];

	// limark
	printf("Available channels:");

	for(int chan_=1; chan_ < MAX_CHANNELS; chan_++) {
		if( is_common_channel(chan_, node_list, node_num) ) {
#ifdef CRP
			if(check_channel_average(host_, chan_) == true && check_channel_variance(host_, chan_) == true)
#endif // if CRP
			{
				channel_list[channel_num]=chan_;
				// limark
				printf(" %d", chan_);
				channel_num++;
			}
		}
	}

	// limark
	printf("\n");

	if(channel_num == 0)
		return -1;

	int channel_;
	double weight_ = MAXD;

	for(int i=0; i < channel_num; i++) {
		int chan_ = channel_list[i];
		double t_ = 0.0;
		for(int j=1; j < node_num; j++) {
			int prev_node_ = node_list[j];	
			t_+=cal_link_wt(host_, prev_node_, chan_, current_time);	
		}

		if(t_ < weight_) {
			weight_ = t_;
			channel_ = chan_;
		}
	}
		
	return channel_;
} 

// Return flow id
int
Repository::read_flow_id(int node, int index) {
	
	return repository_table_rx[node].flow[index];
}

// Return flow dst
int
Repository::read_flow_dst(int index) {

	return repository_table_path[index].dst;
}

// Find out who is the previous hop
int
Repository::find_prev_hop(int node, int flow) {

	int i, pre_hop;
	for(i = 0; i < MAX_HOP; i++) {
		if(repository_table_path[flow].relay[i] == node)
			break;
	}
	
	pre_hop = repository_table_path[flow].relay[i+1];

	return pre_hop;
}

void
Repository::clean_route_channel(int *flow_list, int flow_num) {

	for(int i=0; i < flow_num; i++) {
		// read each flow id
		int flow_id_ = flow_list[i];
		printf(" [Broken Route] Src: %d Dst: %d\n",
				repository_table_path[flow_id_].src, repository_table_path[flow_id_].dst);

		// deal with node as relay - decrease set num and clean flow id
		for(int j = 0; j < MAX_HOP; j++) {
			int relay_node_ = repository_table_path[flow_id_].relay[j];

			if((relay_node_ != -1) && (relay_node_ != repository_table_path[flow_id_].src)) {
				if(repository_table_rx[relay_node_].set > 0) {
					repository_table_rx[relay_node_].set--; // Decrease relay's set number in node table
				}

				for(int k = 0; k < MAX_FLOWS; k++) { // Clean relay's flow id in node table
					if(repository_table_rx[relay_node_].flow[k] == flow_id_)
						repository_table_rx[relay_node_].flow[k] = -1;
				}
			}
		}

		// deal with route - clean relay, src, dst and reset is_on
		for(int j = 0; j < MAX_HOP; j++) {
			repository_table_path[flow_id_].relay[j] = -1;
		}

		repository_table_path[flow_id_].is_on = 0;
		repository_table_path[flow_id_].src = -1;
		repository_table_path[flow_id_].dst = -1;
	}
}

// Clean current route before repairing 
void
Repository::clean_all_route_channel(int node) {
	
	int flow_id, relay_node;
	int flow_cnt = 0;
	int num_flow = repository_table_rx[node].set;

	for(int i = 0; i < MAX_FLOWS; i++) {
		if(repository_table_rx[node].flow[i] != -1) { 

			// find out each flow this node serves
			flow_cnt++;
			flow_id = repository_table_rx[node].flow[i];
			printf(" [Broken Route] Src: %d Dst: %d\n",
					repository_table_path[flow_id].src, repository_table_path[flow_id].dst);

			// deal with each node as relay in this route entry (except the src node)
			for(int j = 0; j < MAX_HOP; j++) {
				relay_node = repository_table_path[flow_id].relay[j];
				// ---  We don't need to deal with src node since its flow table dosen't have this flow_id!
				if((relay_node != -1) && (relay_node != repository_table_path[flow_id].src)) {
					if(repository_table_rx[relay_node].set > 0) {
						repository_table_rx[relay_node].set--; // Decrease relay's set number in node table
					}
					for(int k = 0; k < MAX_FLOWS; k++) { // Clean relay's flow id in node table
						if(repository_table_rx[relay_node].flow[k] == flow_id)
							repository_table_rx[relay_node].flow[k] = -1;
					}
				}
			}

			// Clean this route entry  
			for(int j = 0; j < MAX_HOP; j++) {
				repository_table_path[flow_id].relay[j] = -1;
			}

			repository_table_path[flow_id].is_on = 0;
			repository_table_path[flow_id].src = -1;
			repository_table_path[flow_id].dst = -1;
		}
	}

	// Error Check
	if(flow_cnt != num_flow) {
		printf("\n[!!!WARNING!!!] set in repository_table_rx is not correct while cleaning.\n");
		exit(0);
	}

	// Clean this node
	repository_table_rx[node].set = 0;
	for( int i = 0; i < MAX_FLOWS; i++) 
		repository_table_rx[node].flow[i] = -1;
}

/*************************************
 * Functions for AODV protocols
 *************************************/

int 
Repository::get_path_id(int dst, int src) {  

	int i;
	int id = -1;

	for(i = 0; i < MAX_FLOWS; i++) {

		if(repository_table_path[i].dst == dst && repository_table_path[i].src == src &&
			repository_table_path[i].is_on == 1) { 
			id = i;
			break;	
		}

	}

	return id;
}

int
Repository::get_relay_by_hop(int id, int count) {

	int node_;
	node_ = repository_table_path[id].relay[count];
	return node_;
}

int
Repository::check_path_state(int id) {

	return repository_table_path[id].is_on;
}

#endif //LI_MOD


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Switchable Interface Implementation END

/*
Copyright (c) 1997, 1998 Carnegie Mellon University.  All Rights
Reserved. 

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The AODV code developed by the CMU/MONARCH group was optimized and tuned by Samir Das and Mahesh Marina, University of Cincinnati. The work was partially done in Sun Microsystems. Modified for gratuitous replies by Anant Utgikar, 09/16/02.

*/

//#include <ip.h>

#include <aodv/aodv.h>
#include <aodv/aodv_packet.h>
#include <random.h>
#include <cmu-trace.h>
//#include <energy-model.h>

#define max(a,b)        ( (a) > (b) ? (a) : (b) )
#define CURRENT_TIME    Scheduler::instance().clock()

//#define DEBUG
//#define ERROR

#ifdef DEBUG
static int extra_route_reply = 0;
static int limit_route_request = 0;
static int route_request = 0;
#endif


/*
  TCL Hooks
*/


int hdr_aodv::offset_;
static class AODVHeaderClass : public PacketHeaderClass {
public:
        AODVHeaderClass() : PacketHeaderClass("PacketHeader/AODV",
                                              sizeof(hdr_all_aodv)) {
	  bind_offset(&hdr_aodv::offset_);
	} 
} class_rtProtoAODV_hdr;

static class AODVclass : public TclClass {
public:
        AODVclass() : TclClass("Agent/AODV") {}
        TclObject* create(int argc, const char*const* argv) {
          assert(argc == 5);
          //return (new AODV((nsaddr_t) atoi(argv[4])));
	  return (new AODV((nsaddr_t) Address::instance().str2addr(argv[4])));
        }
} class_rtProtoAODV;


int
AODV::command(int argc, const char*const* argv) {
  if(argc == 2) {
  Tcl& tcl = Tcl::instance();
    
    if(strncasecmp(argv[1], "id", 2) == 0) {
      tcl.resultf("%d", index);
      return TCL_OK;
    }
    
    if(strncasecmp(argv[1], "start", 2) == 0) {
      btimer.handle((Event*) 0);

#ifndef AODV_LINK_LAYER_DETECTION
      htimer.handle((Event*) 0);
	#ifndef LI_MOD
      ntimer.handle((Event*) 0);
	#endif // No LI_MOD
#endif // LINK LAYER DETECTION

      rtimer.handle((Event*) 0);
      return TCL_OK;
     }               
  }
  else if(argc == 3) {
    if(strcmp(argv[1], "index") == 0) {
      index = atoi(argv[2]);
      return TCL_OK;
    }

    else if(strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
      logtarget = (Trace*) TclObject::lookup(argv[2]);
      if(logtarget == 0)
	return TCL_ERROR;
      return TCL_OK;
    }
    else if(strcmp(argv[1], "drop-target") == 0) {
    int stat = rqueue.command(argc,argv);
      if (stat != TCL_OK) return stat;
      return Agent::command(argc, argv);
    }
    else if(strcmp(argv[1], "if-queue") == 0) {
    ifqueue = (PriQueue*) TclObject::lookup(argv[2]);
      
      if(ifqueue == 0)
	return TCL_ERROR;
      return TCL_OK;
    }
    else if (strcmp(argv[1], "port-dmux") == 0) {
    	dmux_ = (PortClassifier *)TclObject::lookup(argv[2]);
	if (dmux_ == 0) {
		fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__,
		argv[1], argv[2]);
		return TCL_ERROR;
	}
	return TCL_OK;
    }
    

    // CRAHNs Model START
    // @author:  Marco Di Felice
  
    else if (strcasecmp (argv[1], "down-target-1") == 0) {
	downtarget_[0] = (NsObject *) TclObject::lookup(argv[2]);
     	return TCL_OK;
    }
    else if (strcasecmp (argv[1], "down-target-2") == 0) {
	downtarget_[1] = (NsObject *) TclObject::lookup(argv[2]);
     	return TCL_OK;
    }
    else if (strcasecmp (argv[1], "down-target-3") == 0) {
	downtarget_[2] = (NsObject *) TclObject::lookup(argv[2]);
     	return TCL_OK;
    }
   
    else if (strcmp(argv[1], "setRepository") == 0) {
	repository_ = (Repository*) TclObject::lookup(argv[2]);
        return (TCL_OK);
    }
  
  // CRAHNs Model END

  }
  return Agent::command(argc, argv);
}

/* 
   Constructor
*/

AODV::AODV(nsaddr_t id) : Agent(PT_AODV),
			  btimer(this), htimer(this), ntimer(this), 
			  rtimer(this), lrtimer(this), rqueue() {
 
                
	index = id;
	seqno = 2; // seqno initialization - Li
	bid = 1;

	LIST_INIT(&nbhead);
	LIST_INIT(&bihead);

	logtarget = 0;
	ifqueue = 0;


	// CRAHNs Model START
	// @author:  Marco Di Felice

	// Initialize the number of interferers for channel ...
	for (int i=0; i<MAX_CHANNELS; i++)
		num_recv_channels_[i]=0;

#ifndef LI_MOD  
	// htimer can be started in command() if 
	// we don't use AODV_LINK_LAYER_DETECTION - Li
	htimer.handle(NULL);
#endif // No LI_MOD

	// Initialize your channel allocation policy here ...
	channel_allocation_mode_=MIN_INTERFERENCE_POLICY;

	num_packets_sent_=0;  

	// CRAHNs Model END

}


/*
  Timers
*/

void
BroadcastTimer::handle(Event*) {
	agent->id_purge();
	Scheduler::instance().schedule(this, &intr, BCAST_ID_SAVE);
}



// CRAHNs Model START
// @author:  Marco Di Felice

// Handle Timer expire events
void
HelloTimer::handle(Event*) {

#ifdef LI_MOD

	if (CURRENT_TIME < 3.0)  {
		agent->sendHello();
		Scheduler::instance().schedule(this, &intr, CURRENT_TIME + Random::uniform()*0.5);
		return;  
	}

	return;

#else // no LI_MOD

	if (CURRENT_TIME < 0.5)  {
		Scheduler::instance().schedule(this, &intr, 0.5 + Random::uniform()*0.5);
		return;
	}

	// Channel Allocation: the fixed receiving channel is decided
	agent->set_receiver_channel();
	// HELLO message is broadcasted every HELLO_REFRESH_TIMER seconds
	#ifdef HELLO_MSG_OPTIMIZATION
	if (agent->num_packets_sent_ > PACKET_ACTIVE_THRESHOLD )
		agent->sendHello();
	#else
		agent->sendHello();
	#endif

	agent->num_packets_sent_=0;

	double interval= (Random::uniform() * 0.5 + HELLO_REFRESH_TIMER);
	assert(interval >= 0);
	Scheduler::instance().schedule(this, &intr, interval);

#endif
}

// CRAHNs Model END
 


void
NeighborTimer::handle(Event*) {
	agent->nb_purge();
	Scheduler::instance().schedule(this, &intr, HELLO_INTERVAL);
}

void
RouteCacheTimer::handle(Event*) {
	agent->rt_purge();
	#define FREQUENCY 0.5 // sec
	Scheduler::instance().schedule(this, &intr, FREQUENCY);
}

void
LocalRepairTimer::handle(Event* p)  {  // SRD: 5/4/99
	aodv_rt_entry *rt;
	struct hdr_ip *ih = HDR_IP( (Packet *)p);

	/* you get here after the timeout in a local repair attempt */
	/*	fprintf(stderr, "%s\n", __FUNCTION__); */


	rt = agent->rtable.rt_lookup(ih->daddr());

	if (rt && rt->rt_flags != RTF_UP) {
		// route is yet to be repaired
		// I will be conservative and bring down the route
		// and send route errors upstream.
		/* The following assert fails, not sure why */
		/* assert (rt->rt_flags == RTF_IN_REPAIR); */

		//rt->rt_seqno++;
		agent->rt_down(rt);
		// send RERR
#ifdef DEBUG
		fprintf(stderr,"Node %d: Dst - %d, failed local repair\n",index, rt->rt_dst);
#endif      
	}
	Packet::free((Packet *)p);
}


/*
	Broadcast ID Management  Functions
 */


void
AODV::id_insert(nsaddr_t id, u_int32_t bid) {
	BroadcastID *b = new BroadcastID(id, bid);

	assert(b);
	b->expire = CURRENT_TIME + BCAST_ID_SAVE;
	LIST_INSERT_HEAD(&bihead, b, link);
}

/* SRD */
bool
AODV::id_lookup(nsaddr_t id, u_int32_t bid) {
	BroadcastID *b = bihead.lh_first;

	// Search the list for a match of source and bid
	for( ; b; b = b->link.le_next) {
		if ((b->src == id) && (b->id == bid))
			return true;     
	}
	return false;
}

void
AODV::id_purge() {
	BroadcastID *b = bihead.lh_first;
	BroadcastID *bn;
	double now = CURRENT_TIME;

	for(; b; b = bn) {
		bn = b->link.le_next;
		if(b->expire <= now) {
			LIST_REMOVE(b,link);
			delete b;
		}
	}
}

/*
  Helper Functions
*/

double
AODV::PerHopTime(aodv_rt_entry *rt) {
	int num_non_zero = 0, i;
	double total_latency = 0.0;

	if (!rt)
		return ((double) NODE_TRAVERSAL_TIME );
	
	for (i=0; i < MAX_HISTORY; i++) {
		if (rt->rt_disc_latency[i] > 0.0) {
			num_non_zero++;
			total_latency += rt->rt_disc_latency[i];
		}
	}
	if (num_non_zero > 0)
		return(total_latency / (double) num_non_zero);
	else
		return((double) NODE_TRAVERSAL_TIME);

}

/*
  Link Failure Management Functions
*/

static void
aodv_rt_failed_callback(Packet *p, void *arg) {
	((AODV*) arg)->rt_ll_failed(p);
}

/*
 * This routine is invoked when the link-layer reports a route failed.
 */
void
AODV::rt_ll_failed(Packet *p) {

#ifndef AODV_LINK_LAYER_DETECTION
	drop(p, DROP_RTR_MAC_CALLBACK);
	// Now, return.
#else
	// Move the following 5 lines from function beginning
	// Just to avoid warning. - Li
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	aodv_rt_entry *rt;
	nsaddr_t broken_nbr = ch->next_hop_;


	/*
	 * Non-data packets and Broadcast Packets can be dropped.
	 */
	if(! DATA_PACKET(ch->ptype()) ||
		(u_int32_t) ih->daddr() == IP_BROADCAST) {
		drop(p, DROP_RTR_MAC_CALLBACK);
		return;
	}
	log_link_broke(p);
	if((rt = rtable.rt_lookup(ih->daddr())) == 0) {
		drop(p, DROP_RTR_MAC_CALLBACK);
		return;
	}
	log_link_del(ch->next_hop_);

	#ifdef AODV_LOCAL_REPAIR
	/* if the broken link is closer to the dest than source, 
	 attempt a local repair. Otherwise, bring down the route. */


	if (ch->num_forwards() > rt->rt_hops) {
		local_rt_repair(rt, p); // local repair
		// retrieve all the packets in the ifq using this link,
		// queue the packets for which local repair is done, 
		return;
	}
	else	
	#endif // LOCAL REPAIR	

	{
		drop(p, DROP_RTR_MAC_CALLBACK);
		// Do the same thing for other packets in the interface queue using the
		// broken link -Mahesh
		while((p = ifqueue->filter(broken_nbr))) {
			drop(p, DROP_RTR_MAC_CALLBACK);
		}	
		nb_delete(broken_nbr); // When AODV_LINK_LAYER_DETECTION is turned on - Li 
	}
#endif // LINK LAYER DETECTION
}

void
AODV::handle_link_failure(nsaddr_t id) {

	aodv_rt_entry *rt, *rtn;
	Packet *rerr = Packet::alloc();
	struct hdr_aodv_error *re = HDR_AODV_ERROR(rerr);

	re->DestCount = 0;
	for(rt = rtable.head(); rt; rt = rtn) {  // for each rt entry
		rtn = rt->rt_link.le_next;
		if ((rt->rt_hops != INFINITY2) && (rt->rt_nexthop == id) ) {

			assert (rt->rt_flags == RTF_UP);
			assert((rt->rt_seqno%2) == 0);
			rt->rt_seqno++;
			re->unreachable_dst[re->DestCount] = rt->rt_dst;
			re->unreachable_dst_seqno[re->DestCount] = rt->rt_seqno;
			#ifdef DEBUG
			fprintf(stderr, "%s(%f): %d\t(%d\t%u\t%d)\n", __FUNCTION__, CURRENT_TIME,
					index, re->unreachable_dst[re->DestCount],
					re->unreachable_dst_seqno[re->DestCount], rt->rt_nexthop);
			#endif // DEBUG
			re->DestCount += 1;
			rt_down(rt);
		}
		// remove the lost neighbor from all the precursor lists
		rt->pc_delete(id);
	}  

	if (re->DestCount > 0) {
		#ifdef DEBUG
		fprintf(stderr, "%s(%f): %d\tsending RERR...\n", __FUNCTION__, CURRENT_TIME, index);
		#endif // DEBUG
		sendError(rerr, false); // be careful about who should handle link failure - Li
	}
	else {
		Packet::free(rerr);
	}
}

void
AODV::local_rt_repair(aodv_rt_entry *rt, Packet *p) {
	#ifdef DEBUG
	fprintf(stderr,"%s: Dst - %d\n", __FUNCTION__, rt->rt_dst); 
	#endif  
	// Buffer the packet 
	rqueue.enque(p);

	// mark the route as under repair 
	rt->rt_flags = RTF_IN_REPAIR;

	sendRequest(rt->rt_dst);

	// set up a timer interrupt
	Scheduler::instance().schedule(&lrtimer, p->copy(), rt->rt_req_timeout);
}

void
AODV::rt_update(aodv_rt_entry *rt, u_int32_t seqnum, u_int16_t metric,
				nsaddr_t nexthop, double expire_time) {

	rt->rt_seqno = seqnum;
	rt->rt_hops = metric;
	rt->rt_flags = RTF_UP;
	rt->rt_nexthop = nexthop;
	rt->rt_expire = expire_time;
}

void
AODV::rt_down(aodv_rt_entry *rt) {
	/*
	 *  Make sure that you don't "down" a route more than once.
	 */

	if(rt->rt_flags == RTF_DOWN) {
		return;
	}

	// assert (rt->rt_seqno%2); // is the seqno odd?
	rt->rt_last_hop_count = rt->rt_hops;
	rt->rt_hops = INFINITY2;
	rt->rt_flags = RTF_DOWN;
	rt->rt_nexthop = 0;
	rt->rt_expire = 0;

} /* rt_down function */

/*
  Route Handling Functions
*/

void
AODV::rt_resolve(Packet *p) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	aodv_rt_entry *rt;

	/*
	 *  Set the transmit failure callback.  That
	 *  won't change.
	 */
	ch->xmit_failure_ = aodv_rt_failed_callback;
	ch->xmit_failure_data_ = (void*) this;

	rt = rtable.rt_lookup(ih->daddr());
	if(rt == 0) {
		rt = rtable.rt_add(ih->daddr());
	}

	/*
	 * If the route is up, forward the packet 
	 */
	
	if(rt->rt_flags == RTF_UP) {
		assert(rt->rt_hops != INFINITY2);
		forward(rt, p, NO_DELAY);
	}
	/*
	 *  if I am the source of the packet, then do a Route Request.
	 */
	else if(ih->saddr() == index) {
		rqueue.enque(p);
		sendRequest(rt->rt_dst);
	}
#ifndef LI_MOD // No LI_MOD
	/*
	 *	A local repair is in progress. Buffer the packet. 
	 */
	else if (rt->rt_flags == RTF_IN_REPAIR) {
		rqueue.enque(p);
	}
#endif
	/*
	 * I am trying to forward a packet for someone else to which
	 * I don't have a route.
	 */
	else {
		Packet *rerr = Packet::alloc();
		struct hdr_aodv_error *re = HDR_AODV_ERROR(rerr);
		/* 
		 * For now, drop the packet and send error upstream.
		 * Now the route errors are broadcast to upstream
		 * neighbors - Mahesh 09/11/99
		 */	
 
		assert (rt->rt_flags == RTF_DOWN);
		re->DestCount = 0;
		re->unreachable_dst[re->DestCount] = rt->rt_dst;
		re->unreachable_dst_seqno[re->DestCount] = rt->rt_seqno;
		re->DestCount += 1;
#ifdef DEBUG
		fprintf(stderr, "%s: sending RERR...\n", __FUNCTION__);
#endif
		sendError(rerr, false);

		drop(p, DROP_RTR_NO_ROUTE);
	}

}

void
AODV::rt_purge() {
	aodv_rt_entry *rt, *rtn;
	double now = CURRENT_TIME;
	double delay = 0.0;
	Packet *p;

	for(rt = rtable.head(); rt; rt = rtn) {  // for each rt entry
		rtn = rt->rt_link.le_next;
		if ((rt->rt_flags == RTF_UP) && (rt->rt_expire < now)) {
			// if a valid route has expired, purge all packets from 
			// send buffer and invalidate the route.                    
			assert(rt->rt_hops != INFINITY2);
			while((p = rqueue.deque(rt->rt_dst))) {
#ifdef DEBUG
				fprintf(stderr, "%s: calling drop()\n",
					   __FUNCTION__);
#endif // DEBUG
				drop(p, DROP_RTR_NO_ROUTE);
			}
			rt->rt_seqno++;
			assert (rt->rt_seqno%2);
			rt_down(rt);
		}
		else if (rt->rt_flags == RTF_UP) {
			// If the route is not expired,
			// and there are packets in the sendbuffer waiting,
			// forward them. This should not be needed, but this extra 
			// check does no harm.
			assert(rt->rt_hops != INFINITY2);
			while((p = rqueue.deque(rt->rt_dst))) {
				forward (rt, p, delay);
				delay += ARP_DELAY;
			}
		} 
		else if (rqueue.find(rt->rt_dst)) {
			// If the route is down and 
			// if there is a packet for this destination waiting in
			// the sendbuffer, then send out route request. sendRequest
			// will check whether it is time to really send out request
			// or not.
			// This may not be crucial to do it here, as each generated 
			// packet will do a sendRequest anyway.

			// Be careful about who comes here - Li
			sendRequest(rt->rt_dst); // rt_purge() calls it - Li
			printf("\n [Route Purge] rt_purge() calls sendRequest() on Node %d \n\n", index);
		}
	}
}

/*
  Packet Reception Routines
*/

void
AODV::recv(Packet *p, Handler*) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);

	assert(initialized());
	//assert(p->incoming == 0);
	// XXXXX NOTE: use of incoming flag has been depracated; In order to track direction of pkt flow, direction_ in hdr_cmn is used instead. see packet.h for details.

	if(ch->ptype() == PT_AODV) {
		ih->ttl_ -= 1;
		recvAODV(p);
		return;
	}
 
	// CRAHNs Model START
	// @author:  Marco Di Felice

	if(ch->ptype() == PT_NOTIFICATION) {
		recvNotification(p);
		return;
	}

	// CRAHNs Model END
	// @author:  Marco Di Felice
 
	/*
	 *  Must be a packet I'm originating...
	 */
	if((ih->saddr() == index) && (ch->num_forwards() == 0)) {
		/*
		 * Add the IP Header
		 */
		ch->size() += IP_HDR_LEN;
		// Added by Parag Dadhania && John Novatnack to handle broadcasting
		if ( (u_int32_t)ih->daddr() != IP_BROADCAST)
			ih->ttl_ = NETWORK_DIAMETER;
	}
	/*
	 *  I received a packet that I sent.  Probably
	 *  a routing loop.
	 */
	else if(ih->saddr() == index) {
		drop(p, DROP_RTR_ROUTE_LOOP);
		return;
	}
	/*
	 *  Packet I'm forwarding...
	 */
	else {
		/*
		 *  Check the TTL.  If it is zero, then discard.
		 */
		if(--ih->ttl_ == 0) {
			drop(p, DROP_RTR_TTL);
			return;
		}
	}

	// Added by Parag Dadhania && John Novatnack to handle broadcasting
	if ( (u_int32_t)ih->daddr() != IP_BROADCAST)
		rt_resolve(p);
	else
		forward((aodv_rt_entry*) 0, p, NO_DELAY);
}


void
AODV::recvAODV(Packet *p) {
	struct hdr_aodv *ah = HDR_AODV(p);

	assert(HDR_IP (p)->sport() == RT_PORT);
	assert(HDR_IP (p)->dport() == RT_PORT);

	/*
	 * Incoming Packets.
	 */
	switch(ah->ah_type) {

	case AODVTYPE_RREQ:
		recvRequest(p);
		break;

	case AODVTYPE_RREP:
		recvReply(p);
		break;

	case AODVTYPE_RERR:
		recvError(p);
		break;

	case AODVTYPE_HELLO:
		recvHello(p);
		break;
        
	default:
		fprintf(stderr, "Invalid AODV type (%x)\n", ah->ah_type);
		exit(1);
	}

}


void
AODV::recvRequest(Packet *p) {
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);
	aodv_rt_entry *rt;

	/*
	 * Drop if:
	 *      - I'm the source
	 *      - I recently heard this request.
	 */

	if(rq->rq_src == index) {
		#ifdef DEBUG
		fprintf(stderr, "%s: got my own REQUEST\n", __FUNCTION__);
		#endif // DEBUG
		Packet::free(p);
		return;
	} 

 	if (id_lookup(rq->rq_src, rq->rq_bcast_id)) {

		#ifdef DEBUG
		fprintf(stderr, "%s: discarding request\n", __FUNCTION__);
		#endif // DEBUG
	 
		Packet::free(p);
		return;
 	}

 	/*
  	 * Cache the broadcast ID
  	 */

 	id_insert(rq->rq_src, rq->rq_bcast_id);

	/* 
	 * We are either going to forward the REQUEST or generate a
	 * REPLY. Before we do anything, we make sure that the REVERSE
	 * route is in the route table.
	 */

	aodv_rt_entry *rt0; // rt0 is the reverse route 
   
	rt0 = rtable.rt_lookup(rq->rq_src);
	if(rt0 == 0) { /* if not in the route tabe */
		// create an entry for the reverse route.
		rt0 = rtable.rt_add(rq->rq_src);
	}
  
	rt0->rt_expire = max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE));

	if ( (rq->rq_src_seqno > rt0->rt_seqno ) ||
		((rq->rq_src_seqno == rt0->rt_seqno) && 
		 (rq->rq_hop_count < rt0->rt_hops)) ) {
		// If we have a fresher seq no. or lesser #hops for the 
		// same seq no., update the rt entry. Else don't bother.
		rt_update(rt0, rq->rq_src_seqno, rq->rq_hop_count, ih->saddr(),
					max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE)) );
		if (rt0->rt_req_timeout > 0.0) {
			// Reset the soft state and 
			// Set expiry time to CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT
			// This is because route is used in the forward direction,
			// but only sources get benefited by this change
			rt0->rt_req_cnt = 0;
			rt0->rt_req_timeout = 0.0; 
			rt0->rt_req_last_ttl = rq->rq_hop_count;
			rt0->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
		}

		/* Find out whether any buffered packet can benefit from the 
		 * reverse route.
		 * May need some change in the following code - Mahesh 09/11/99
		 */
		assert (rt0->rt_flags == RTF_UP);
		Packet *buffered_pkt;
		while ((buffered_pkt = rqueue.deque(rt0->rt_dst))) {
			if (rt0 && (rt0->rt_flags == RTF_UP)) {
				assert(rt0->rt_hops != INFINITY2);
				forward(rt0, buffered_pkt, NO_DELAY);
			}
		}
	} 
	// End for putting reverse route in rt table


	/*
	 * We have taken care of the reverse route stuff.
	 * Now see whether we can send a route reply. 
	 */

	rt = rtable.rt_lookup(rq->rq_dst);

	// First check if I am the destination ..

	if(rq->rq_dst == index) {

		#ifdef DEBUG
		fprintf(stderr, "%d - %s: destination sending reply\n",
				index, __FUNCTION__);
		#endif // DEBUG

		   
		// Just to be safe, I use the max. Somebody may have
		// incremented the dst seqno.
		seqno = max(seqno, rq->rq_dst_seqno)+1; // seqno increases when recvRequest() - Li
		if (seqno%2) seqno++;
		
		#ifdef LI_MOD
		printf("[RREQ RECV] Node %d received RREQ at time %f and is going to call set_route_channel() \n", index, CURRENT_TIME);
		if (repository_->set_route_channel(rq->rq_src, index, CURRENT_TIME) != 0) {
			printf("\n!!! Destination %d cannot find the path to Source %d !!!\n\n", index, rq->rq_src);
			return;
		}
		#endif //LI_MOD

		sendReply(rq->rq_src,           // IP Destination
			1,                    // Hop Count
			index,                // Dest IP Address
			seqno,                // Dest Sequence Num
			MY_ROUTE_TIMEOUT,     // Lifetime
			rq->rq_timestamp);    // timestamp

		Packet::free(p);

	} //end if(rq->rq_dst == index)


#ifndef LI_MOD // We don't need relay to reply - Li
	// I am not the destination, but I may have a fresh enough route.

	else if (rt && (rt->rt_hops != INFINITY2) && 
			(rt->rt_seqno >= rq->rq_dst_seqno) ) {

		//assert (rt->rt_flags == RTF_UP);
		assert(rq->rq_dst == rt->rt_dst);
   		//assert ((rt->rt_seqno%2) == 0);	// is the seqno even?
		sendReply(rq->rq_src,
			rt->rt_hops + 1,
			rq->rq_dst,
			rt->rt_seqno,
			(u_int32_t) (rt->rt_expire - CURRENT_TIME),
			//rt->rt_expire - CURRENT_TIME,
			rq->rq_timestamp);
		// Insert nexthops to RREQ source and RREQ destination in the
		// precursor lists of destination and source respectively
		rt->pc_insert(rt0->rt_nexthop); // nexthop to RREQ source
		rt0->pc_insert(rt->rt_nexthop); // nexthop to RREQ destination
	
		#ifdef RREQ_GRAT_RREP  
		sendReply(rq->rq_dst,
			rq->rq_hop_count,
			rq->rq_src,
			rq->rq_src_seqno,
			(u_int32_t) (rt->rt_expire - CURRENT_TIME),
			//rt->rt_expire - CURRENT_TIME,
			rq->rq_timestamp);
		#endif //RREQ_GRAT_RREP
	   
		// TODO: send grat RREP to dst if G flag set in RREQ using rq->rq_src_seqno, rq->rq_hop_counT
	   
		// DONE: Included gratuitous replies to be sent as per IETF aodv draft specification. 
		//As of now, G flag has not been dynamically used and is always set or reset in aodv-packet.h --- Anant Utgikar, 09/16/02.
	
		Packet::free(p);
 	}

#endif //No LI_MOD


	/*
	 * Can't reply. So forward the Route Request
	 */
	else {
		ih->saddr() = index;
		ih->daddr() = IP_BROADCAST;
		rq->rq_hop_count += 1;
		// Maximum sequence number seen en route
		if (rt) rq->rq_dst_seqno = max(rt->rt_seqno, rq->rq_dst_seqno);
		forward((aodv_rt_entry*) 0, p, DELAY);
	}

}


void
AODV::recvReply(Packet *p) {
	//struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
	aodv_rt_entry *rt;
	char suppress_reply = 0;
	double delay = 0.0;
	
#ifdef DEBUG
	fprintf(stderr, "%d - %s: received a REPLY\n", index, __FUNCTION__);
#endif // DEBUG


	/*
	 *  Got a reply. So reset the "soft state" maintained for 
	 *  route requests in the request table. We don't really have
	 *  have a separate request table. It is just a part of the
	 *  routing table itself. 
	 */
	// Note that rp_dst is the dest of the data packets, not the
	// the dest of the reply, which is the src of the data packets.

	rt = rtable.rt_lookup(rp->rp_dst);
        
	/*
	 *  If I don't have a rt entry to this host... adding
	 */
	if(rt == 0) {
		rt = rtable.rt_add(rp->rp_dst);
	}

	/*
	 * Add a forward route table entry... here I am following 
	 * Perkins-Royer AODV paper almost literally - SRD 5/99
	 */

#ifndef LI_MOD // No LI_MOD
	if ( (rt->rt_seqno < rp->rp_dst_seqno) ||   // newer route 
		((rt->rt_seqno == rp->rp_dst_seqno) &&  
		(rt->rt_hops > rp->rp_hop_count)) ) { // shorter or better route
#else // LI_MOD
	// We only care about newer route (with higher destination sequnce number)
	// since the metric is not about hops anymore - Li
	if ( rt->rt_seqno < rp->rp_dst_seqno ) {
#endif
		
		// Update the rt entry 
		rt_update(rt, rp->rp_dst_seqno, rp->rp_hop_count,
					rp->rp_src, CURRENT_TIME + rp->rp_lifetime);

		// reset the soft state
		rt->rt_req_cnt = 0;
		rt->rt_req_timeout = 0.0; 
		rt->rt_req_last_ttl = rp->rp_hop_count;

		// If I am the original source  
		if (ih->daddr() == index) { 
			// Update the route discovery latency statistics
			// rp->rp_timestamp is the time of request origination		
			rt->rt_disc_latency[(unsigned char)rt->hist_indx] = (CURRENT_TIME - rp->rp_timestamp)
											 / (double) rp->rp_hop_count;
			// increment indx for next time
			rt->hist_indx = (rt->hist_indx + 1) % MAX_HISTORY;
		}	

		/*
		* Send all packets queued in the sendbuffer destined for
		* this destination. 
		* XXX - observe the "second" use of p.
		*/
		Packet *buf_pkt;
		while((buf_pkt = rqueue.deque(rt->rt_dst))) {
				if(rt->rt_hops != INFINITY2) {
				assert (rt->rt_flags == RTF_UP);
					// Delay them a little to help ARP. Otherwise ARP 
					// may drop packets. -SRD 5/23/99
					forward(rt, buf_pkt, delay);
					delay += ARP_DELAY;
				}
		}
	}
#ifndef LI_MOD
	// When Destination Sequence Number is not new ...
	else {
		suppress_reply = 1;
	}


	/*
	 * If reply is for me, discard it.
	 */
	if(ih->daddr() == index || suppress_reply) {
		Packet::free(p);
	}
#endif // No LI_MOD

#ifdef LI_MOD
	if(ih->daddr() == index ) {
		Packet::free(p);
	}
#endif // LI_MOD

	/*
	 * Otherwise, forward the Route Reply.
	 */
	else {

#ifndef LI_MOD
		// Find the rt entry
		aodv_rt_entry *rt0 = rtable.rt_lookup(ih->daddr());
		// If the rt is up, forward
		if(rt0 && (rt0->rt_hops != INFINITY2)) {
			assert (rt0->rt_flags == RTF_UP);
			rp->rp_hop_count += 1;
			rp->rp_src = index;
			forward(rt0, p, NO_DELAY);
			// Insert the nexthop towards the RREQ source to
			// the precursor list of the RREQ destination
			rt->pc_insert(rt0->rt_nexthop); // nexthop to RREQ source
		}
		else {
			// I don't know how to forward .. drop the reply. 
			#ifdef DEBUG
				fprintf(stderr, "%s: dropping Route Reply\n", __FUNCTION__);
			#endif // DEBUG
				drop(p, DROP_RTR_NO_ROUTE);
		}
#endif // No LI_MOD

#ifdef LI_MOD
		// Hop is used to check wether I should be the relay
		int path_id = rp->p_id;
		int hop_here = rp->rp_hop_count;
		int my_addr = repository_->get_relay_by_hop(path_id, hop_here);

		// If PU appears during forwarding RREP, drop it.
		if(repository_->check_path_state(path_id) == 0) {
			Packet::free(p);
			return;
		}

		if(my_addr == index) { //check wether I appear in the relay table

			rp->rp_hop_count += 1;		
			rp->rp_src = index;
			forward_rrep(p);

			int next_hop = rp->rp_hop_count; //next hop
			int next_addr = repository_->get_relay_by_hop(path_id, next_hop);
			
			rt->pc_insert(next_addr);

		} else {
			printf("\n [!!!WARNING!!!] RREP FORWARDING WRONG]\n\n");
			drop(p, DROP_RTR_NO_ROUTE);
			exit(0);
		}
#endif // LI_MOD

	} // end forward Route Reply 
} 


void
AODV::recvError(Packet *p) {

	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_error *re = HDR_AODV_ERROR(p);
	aodv_rt_entry *rt;
	u_int8_t i;

	Packet *rerr = Packet::alloc();
	struct hdr_aodv_error *nre = HDR_AODV_ERROR(rerr);
#ifdef LI_MOD // LI_MOD
	Packet *rerr_2 = Packet::alloc();
	struct hdr_aodv_error *nre_2 = HDR_AODV_ERROR(rerr_2);
	Packet *rerr_3 = Packet::alloc();
	struct hdr_aodv_error *nre_3 = HDR_AODV_ERROR(rerr_3);
#endif


	nre->DestCount = 0;
#ifdef LI_MOD // LI_MOD
	nre_2->DestCount = 0;
	nre_3->DestCount = 0;
#endif

	for (i=0; i<re->DestCount; i++) {
		// For each unreachable destination
		rt = rtable.rt_lookup(re->unreachable_dst[i]);
		if ( rt && (rt->rt_hops != INFINITY2) &&
			(rt->rt_nexthop == ih->saddr()) &&
			(rt->rt_seqno <= re->unreachable_dst_seqno[i]) ) {
				assert(rt->rt_flags == RTF_UP);
				assert((rt->rt_seqno%2) == 0); // is the seqno even?
				#ifdef DEBUG
				fprintf(stderr, "%s(%f): %d\t(%d\t%u\t%d)\t(%d\t%u\t%d)\n", __FUNCTION__,CURRENT_TIME,
						index, rt->rt_dst, rt->rt_seqno, rt->rt_nexthop,
						re->unreachable_dst[i],re->unreachable_dst_seqno[i],
						ih->saddr());
				#endif // DEBUG
				rt->rt_seqno = re->unreachable_dst_seqno[i];
				rt_down(rt);

				// Not sure whether this is the right thing to do
				Packet *pkt;
				while((pkt = ifqueue->filter(ih->saddr()))) {
					drop(pkt, DROP_RTR_MAC_CALLBACK);
				}

				// if precursor list non-empty add to RERR and delete the precursor list
				if (!rt->pc_empty()) {
					nre->unreachable_dst[nre->DestCount] = rt->rt_dst;
					nre->unreachable_dst_seqno[nre->DestCount] = rt->rt_seqno;
					nre->DestCount += 1;
#ifdef LI_MOD // LI_MOD
					nre_2->unreachable_dst[nre_2->DestCount] = rt->rt_dst;
					nre_2->unreachable_dst_seqno[nre_2->DestCount] = rt->rt_seqno;
					nre_2->DestCount += 1;
					nre_3->unreachable_dst[nre_3->DestCount] = rt->rt_dst;
					nre_3->unreachable_dst_seqno[nre_3->DestCount] = rt->rt_seqno;
					nre_3->DestCount += 1;
#endif
					rt->pc_delete();
				}
		}
	} 

	if (nre->DestCount > 0) {
		#ifdef DEBUG
		fprintf(stderr, "%s(%f): %d\t sending RERR...\n", __FUNCTION__, CURRENT_TIME, index);
		#endif // DEBUG
		sendError(rerr);
#ifdef LI_MOD // LI_MOD
		sendError(rerr_2, true);
		sendError(rerr_3, true);
#endif
		// limark
		printf(" [error] Node %d receive and send RRER, Time %f\n", index, CURRENT_TIME);
	}
	else {
		Packet::free(rerr);
#ifdef LI_MOD // LI_MOD
		Packet::free(rerr_2);
		Packet::free(rerr_3);
#endif
	}

	Packet::free(p);
}


/*
   Packet Transmission Routines
*/

void
AODV::forward(aodv_rt_entry *rt, Packet *p, double delay) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);

	if(ih->ttl_ == 0) {

#ifdef DEBUG
		fprintf(stderr, "%s: calling drop()\n", __PRETTY_FUNCTION__);
#endif // DEBUG
 
		drop(p, DROP_RTR_TTL);
		return;
	}


	if ((ch->ptype() != PT_AODV && ch->direction() == hdr_cmn::UP &&
		((u_int32_t)ih->daddr() == IP_BROADCAST))
		|| (ih->daddr() == here_.addr_)) { // Add parentheses to avoid warning - Li
			dmux_->recv(p,0);
			return;
	}

	if (rt) {
		assert(rt->rt_flags == RTF_UP);
		rt->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
		ch->next_hop_ = rt->rt_nexthop;
		ch->addr_type() = NS_AF_INET;
		ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
	}
	else { // if it is a broadcast packet
		// assert(ch->ptype() == PT_AODV); // maybe a diff pkt type like gaf
		assert(ih->daddr() == (nsaddr_t) IP_BROADCAST);
		ch->addr_type() = NS_AF_NONE;
		ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
	}

	// If it is a broadcast packet - Li
	if (ih->daddr() == (nsaddr_t) IP_BROADCAST) {
		assert(rt == 0);
		/*
		 *  Jitter the sending of broadcast packets by 10ms
		 */
		Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p,
									   0.01 * Random::uniform());
	} 
	// If it is NOT a broadcast packet - Li
	else { 
		// when delay > 0.0 - Li
		if(delay > 0.0) {
			if (ch->ptype() == PT_AODV) {
				ch->channel_=CONTROL_CHANNEL;	   

				// Broadcast messages are sent on the CONTROL_RADIO
				Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, delay);
			}
			else {
		
				num_packets_sent_++;
				ch->channel_=repository_->get_recv_channel(rt->rt_nexthop);
				
				// Update sending channel - Li	
				repository_->update_send_channel(index, ch->channel_,CURRENT_TIME);

				// Default case: use the TRANSMITTER interface 
				Scheduler::instance().schedule(downtarget_[TRANSMITTER_RADIO], p, delay);
			}
		}
		// when delay = 0.0 - Li
		else {
			// Not a broadcast packet, no delay, send immediately
			if (ch->ptype() == PT_AODV) {
				ch->channel_=CONTROL_CHANNEL;

				// Broadcast messages are sent on the CONTROL_RADIO
				Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.);
			}
			else {

				ch->channel_=repository_->get_recv_channel(rt->rt_nexthop);

				// Update sending channel - Li	
				repository_->update_send_channel(index, ch->channel_,CURRENT_TIME);

				num_packets_sent_++;

				// Default case: use the TRANSMITTER interface 
				Scheduler::instance().schedule(downtarget_[TRANSMITTER_RADIO], p, 0.);

			}
		}
	}
}

#ifdef LI_MOD
void
AODV::forward_rrep( Packet *p ) {

	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);

	int path_id = rp->p_id;
	int hop_forward = rp->rp_hop_count; // hop_forward is the next hop number
	
	ch->next_hop_ = repository_->get_relay_by_hop(path_id, hop_forward);
   	ch->addr_type() = NS_AF_INET;
	ch->direction() = hdr_cmn::DOWN;
	ch->channel_ = CONTROL_CHANNEL;

	Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.);

}
#endif // LI_MOD

void
AODV::sendRequest(nsaddr_t dst) {

	// Allocate a RREQ packet 
	Packet *p = Packet::alloc();
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);
	aodv_rt_entry *rt = rtable.rt_lookup(dst);

	assert(rt);

	/*
	 *  Rate limit sending of Route Requests. We are very conservative
	 *  about sending out route requests. 
	 */

	if (rt->rt_flags == RTF_UP) {
		assert(rt->rt_hops != INFINITY2);
		Packet::free((Packet *)p);
		return;
	}

	if (rt->rt_req_timeout > CURRENT_TIME) {
		Packet::free((Packet *)p);
		return;
	}

	// rt_req_cnt is the no. of times we did network-wide broadcast
	// RREQ_RETRIES is the maximum number we will allow before 
	// going to a long timeout.

	if (rt->rt_req_cnt > RREQ_RETRIES) {
		rt->rt_req_timeout = CURRENT_TIME + MAX_RREQ_TIMEOUT;
		rt->rt_req_cnt = 0;
		Packet *buf_pkt;
		while ((buf_pkt = rqueue.deque(rt->rt_dst))) {
			drop(buf_pkt, DROP_RTR_NO_ROUTE);
		}
		Packet::free((Packet *)p);
		return;
	}

#ifdef DEBUG
	fprintf(stderr, "(%2d) - %2d sending Route Request, dst: %d\n",
			++route_request, index, rt->rt_dst);
#endif // DEBUG

#ifdef LI_MOD
	printf("\n [RREQ] Node %d is going to send RREQ at time %f \n\n", index, CURRENT_TIME);
#endif

	// Determine the TTL to be used this time. 
	// Dynamic TTL evaluation - SRD

	rt->rt_req_last_ttl = max(rt->rt_req_last_ttl,rt->rt_last_hop_count);

	if (0 == rt->rt_req_last_ttl) {
		// first time query broadcast
		ih->ttl_ = TTL_START;
	}
	else {
		// Expanding ring search.
		if (rt->rt_req_last_ttl < TTL_THRESHOLD)
			ih->ttl_ = rt->rt_req_last_ttl + TTL_INCREMENT;
		else {
			// network-wide broadcast
			ih->ttl_ = NETWORK_DIAMETER;
			rt->rt_req_cnt += 1;
		}
	}

	// remember the TTL used  for the next time
	rt->rt_req_last_ttl = ih->ttl_;

	// PerHopTime is the roundtrip time per hop for route requests.
	// The factor 2.0 is just to be safe .. SRD 5/22/99
	// Also note that we are making timeouts to be larger if we have 
	// done network wide broadcast before. 

	rt->rt_req_timeout = 2.0 * (double) ih->ttl_ * PerHopTime(rt); 
	if (rt->rt_req_cnt > 0)
		rt->rt_req_timeout *= rt->rt_req_cnt;
	rt->rt_req_timeout += CURRENT_TIME;

	// Don't let the timeout to be too large, however .. SRD 6/8/99
	if (rt->rt_req_timeout > CURRENT_TIME + MAX_RREQ_TIMEOUT)
		rt->rt_req_timeout = CURRENT_TIME + MAX_RREQ_TIMEOUT;
	rt->rt_expire = 0;

#ifdef DEBUG
	fprintf(stderr, "(%2d) - %2d sending Route Request, dst: %d, tout %f ms\n",
			++route_request, 
			index, rt->rt_dst, 
			rt->rt_req_timeout - CURRENT_TIME);
#endif	// DEBUG
	

	// Fill out the RREQ packet 
	// ch->uid() = 0;
	ch->ptype() = PT_AODV;
	ch->size() = IP_HDR_LEN + rq->size();
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_NONE;
	ch->prev_hop_ = index;          // AODV hack
	ch->channel_ = CONTROL_CHANNEL;

	ih->saddr() = index;
	ih->daddr() = IP_BROADCAST;
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;

	// Fill up some more fields. 
	rq->rq_type = AODVTYPE_RREQ;
	rq->rq_hop_count = 1;
	rq->rq_bcast_id = bid++; // broadcast id - Li
	rq->rq_dst = dst;
	rq->rq_dst_seqno = (rt ? rt->rt_seqno : 0);
	rq->rq_src = index;
	seqno += 2; // seqno increases when sendRequest() - Li
	assert ((seqno%2) == 0);
	rq->rq_src_seqno = seqno; // source sequence number - Li
	rq->rq_timestamp = CURRENT_TIME;

	Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.);

}

void
AODV::sendReply(nsaddr_t ipdst, u_int32_t hop_count, nsaddr_t rpdst,
                u_int32_t rpseq, u_int32_t lifetime, double timestamp) {

	Packet *p = Packet::alloc();
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
	aodv_rt_entry *rt = rtable.rt_lookup(ipdst);

	#ifdef DEBUG
	fprintf(stderr, "sending Reply from %d at %.2f\n", index, Scheduler::instance().clock());
	#endif // DEBUG
	assert(rt);

	#ifdef LI_MOD
	//send rrep with the best path
	int path_id = repository_->get_path_id(index, ipdst);
	if(path_id != -1) // One flow's dst node can't be another's
		rp->p_id = path_id;
	else {
		Packet::free((Packet *)p);
		printf("\n [!!!WARNING!!!] Node %d can't find path id.\n\n", index);
		exit(0);
	}
	#endif //LI_MOD

 	rp->rp_type = AODVTYPE_RREP;
 	//rp->rp_flags = 0x00;
 	rp->rp_hop_count = hop_count;
 	rp->rp_dst = rpdst;//requested dest
 	rp->rp_dst_seqno = rpseq;
 	rp->rp_src = index;
 	rp->rp_lifetime = lifetime;
 	rp->rp_timestamp = timestamp;
   
 	// ch->uid() = 0;
 	ch->ptype() = PT_AODV;
 	ch->size() = IP_HDR_LEN + rp->size();
 	ch->iface() = -2;
 	ch->error() = 0;
 	ch->addr_type() = NS_AF_INET;
	#ifndef LI_MOD
 	ch->next_hop_ = rt->rt_nexthop;
	#endif //no LI_MOD

	#ifdef LI_MOD
	ch->next_hop_ = repository_->get_relay_by_hop(path_id, hop_count); //add
	(void)rt; // Just to avoid warning - Li
	#endif //LI_MOD

 	ch->prev_hop_ = index;          // AODV hack
 	ch->direction() = hdr_cmn::DOWN;
 	ch->channel_ = CONTROL_CHANNEL;


 	ih->saddr() = index;
 	ih->daddr() = ipdst;
 	ih->sport() = RT_PORT;
 	ih->dport() = RT_PORT;
 	ih->ttl_ = NETWORK_DIAMETER;

 	Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.);

}

void
AODV::sendError(Packet *p, bool jitter) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_error *re = HDR_AODV_ERROR(p);
    
#ifdef ERROR
	fprintf(stderr, "sending Error from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG

	re->re_type = AODVTYPE_RERR;
	//re->reserved[0] = 0x00; re->reserved[1] = 0x00;
	// DestCount and list of unreachable destinations are already filled

	// ch->uid() = 0;
	ch->ptype() = PT_AODV;
	ch->size() = IP_HDR_LEN + re->size();
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_NONE;
	ch->next_hop_ = 0;
	ch->prev_hop_ = index;          // AODV hack
	ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction

	ih->saddr() = index;
	ih->daddr() = IP_BROADCAST;
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 1;

	// Do we need any jitter? Yes
	if (jitter)
#ifndef LI_MOD // No LI_MOD
		Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.01*Random::uniform());
#else // LI_MOD
		// Increase random delay scale from 0.01 to 0.02
		Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.02*Random::uniform());
#endif 
	else
		Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.0);

}


/*
   Neighbor Management Functions
*/

void
AODV::sendHello() {

	Packet *p = Packet::alloc();
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_hello *rh = HDR_AODV_HELLO(p);

#ifdef DEBUG
	fprintf(stderr, "sending Hello from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG

	rh->rp_type = AODVTYPE_HELLO;
	rh->rp_hop_count = 1;
	rh->rp_dst = index;

	// CRAHNs Model START
	// @author:  Marco Di Felice

	// Includes current receiving channel in the HELLO message
	rh->rp_channel = repository_->get_recv_channel(index);

	// Include channel information for neighbouring nodes (1-hop neighbours)
	AODV_Neighbor *nb = nbhead.lh_first;
	int counter_neighbours=0;

	for (int i=0; i<MAX_HELLO_NEIGHBOURS; i++) {
		if ((nb) && (nb->hop==1)) {	
			rh->rp_neighbour_table[counter_neighbours].id= nb->nb_addr;
			rh->rp_neighbour_table[counter_neighbours].channel=nb->channel;
			counter_neighbours++;		
		}
		
		if (nb)
			nb = nb->nb_link.le_next;
	}
 
	// Fill the table with -1 values
	for (int i=counter_neighbours; i<MAX_HELLO_NEIGHBOURS; i++) {
		rh->rp_neighbour_table[i].id= -1;
		rh->rp_neighbour_table[i].channel=-1;
		
	}

	// CRAHNs Model END

	ch->ptype() = PT_AODV;
	ch->size() = IP_HDR_LEN + rh->size();
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_NONE;
	ch->prev_hop_ = index;          // AODV hack

	ih->saddr() = index;
	ih->daddr() = IP_BROADCAST;
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 1;

	Scheduler::instance().schedule(downtarget_[CONTROL_RADIO], p, 0.0);
}


void
AODV::recvHello(Packet *p) {
	//struct hdr_ip *ih = HDR_IP(p);
	struct hdr_aodv_hello *rp = HDR_AODV_HELLO(p);
	AODV_Neighbor *nb;
 
	// CRAHNs Model START
	// @author:  Marco Di Felice

#ifdef CHANNEL_DEBUG
	printf("---------------------------------------------------- \n");
	printf(" [HELLO RECEIVED] Node: %d Neighbour 1-hop: %d Channel: %d \n", index, rp->rp_dst, rp->rp_channel); 
#endif 

	update_neighbourhood(rp->rp_dst,rp->rp_channel,1);

#ifdef LI_MOD
	repository_->update_nb(index, rp->rp_dst);
#endif

	// Add 2-hop neighbours information
	for (int i=0; i<MAX_HELLO_NEIGHBOURS; i++) {
		if ((rp->rp_neighbour_table[i].id >=0) && (rp->rp_neighbour_table[i].id != index)) {
			update_neighbourhood(rp->rp_neighbour_table[i].id,rp->rp_neighbour_table[i].channel,2);
#ifdef CHANNEL_DEBUG
			printf(" [CHANNEL TABLE] Node %d Neighbour %d Channel: %d \n", index,rp->rp_neighbour_table[i].id, rp->rp_neighbour_table[i].channel); 
#endif 
		}
	}
	
#ifdef CHANNEL_DEBUG
	printf(" ---------------------------------------------------- \n");
#endif 
	// CRAHNs Model END

	Packet::free(p);

#ifdef LI_MOD
	(void)nb; // Just to avoid warning - Li
#endif

}




// CRAHNs Model START
// @author:  Marco Di Felice
// Insert a 1-hop or 2-hop neighbour in the Neighbour Table of AODV
void 
AODV::update_neighbourhood(int id, int channel, int hop) {
	AODV_Neighbor *nb;
	nb = nb_lookup(id);

	if(nb == 0) {
		nb_insert(id);
		nb = nb_lookup(id);
	}
 
	else 
		nb->nb_expire = CURRENT_TIME + (1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);

	// Update channel and hop informatio
	nb->channel=channel;
	nb->hop=hop;
}

// CRAHNs Model END

void
AODV::nb_insert(nsaddr_t id) {
	AODV_Neighbor *nb = new AODV_Neighbor(id);

	assert(nb);
	nb->nb_expire = CURRENT_TIME +
					(1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);
	LIST_INSERT_HEAD(&nbhead, nb, nb_link);
	// seqno increases when nb_insert() - Li
	seqno += 2;             // set of neighbors changed
	assert ((seqno%2) == 0);
}


AODV_Neighbor*
AODV::nb_lookup(nsaddr_t id) {
	AODV_Neighbor *nb = nbhead.lh_first;

	for(; nb; nb = nb->nb_link.le_next) {
		if(nb->nb_addr == id) break;
	}
	return nb;
}


/*
 * Called when we receive *explicit* notification that a Neighbor
 * is no longer reachable.
 */
void
AODV::nb_delete(nsaddr_t id) {
	AODV_Neighbor *nb = nbhead.lh_first;

	log_link_del(id);
	// seqno increases when nb_delete() - Li
	seqno += 2;     // Set of neighbors changed
	assert ((seqno%2) == 0);

	for(; nb; nb = nb->nb_link.le_next) {
		if(nb->nb_addr == id) {
			LIST_REMOVE(nb,nb_link);
			delete nb;
			break;
		}
	}

	handle_link_failure(id);
}


/*
 * Purges all timed-out Neighbor Entries - runs every
 * HELLO_INTERVAL * 1.5 seconds.
 */
void
AODV::nb_purge() {
	AODV_Neighbor *nb = nbhead.lh_first;
	AODV_Neighbor *nbn;
	double now = CURRENT_TIME;

	for(; nb; nb = nbn) {
		nbn = nb->nb_link.le_next;
		if(nb->nb_expire <= now) {
			nb_delete(nb->nb_addr);
		}
	}
}

 // CRAHNs Model START
 // @author:  Marco Di Felice
 
void
AODV::set_receiver_channel() {

	int num_feasible_channels=0;
	int feasible_channels[MAX_CHANNELS];
	bool need_to_switch=false;
	int current_channel= repository_->get_recv_channel(index);
	int minimum;

	switch(channel_allocation_mode_) {
		
		// MIN_INTERFERERS policy: prefer the channel which is less used by 1- and 2- hops neighbouring nodes
		case MIN_INTERFERENCE_POLICY:
			
			// Compute number of receivers on each channel
			compute_receivers_for_channels();
			// Get the number of receivers on the best channel
			minimum=num_recv_channels_[get_less_interfered_channel()];							

			for (int i=1; i<MAX_CHANNELS; i++) {
				if (num_recv_channels_[i] == minimum) {

					// Maybe there might be more than one option ...
					feasible_channels[num_feasible_channels]=i;
					num_feasible_channels++;

					// Channel switching is performed if the new channel is less interfered for a CONVENIENCE_THRESHOLD factor. at least
					//printf("Num receivers on current channel %d next %d \n",num_recv_channels_[current_channel], num_recv_channels_[i]);
					if ((num_recv_channels_[current_channel] - num_recv_channels_[i]) >= CONVENIENCE_THRESHOLD)
						need_to_switch=true;		
				}
			}
			
			// There is a less-interfered channel which can be used ...		
			if (need_to_switch) {
				// Randomly choose between less-used channels
				int channel=((int)(Random::uniform()*num_feasible_channels));		
				if (channel >= num_feasible_channels)
					channel = num_feasible_channels - 1;

				#ifdef CHANNEL_DEBUG
				printf("[ CHANNEL SELECTION ] NODE: %d CHANNEL: %d \n",index,feasible_channels[channel]);
				#endif			
				
				// Update global data structure with channel decision
				repository_->set_recv_channel(index,feasible_channels[channel]);
			}
			
			break;	

		// Other channel allocation policies can be defined here ...
		// IMPLEMENT HERE your own policy
	}
	
}




#define MAXIMUM_NODES 200
// compute_receivers_for_channel: Compute the number of current receivers for each channel
// Return the ID of the channel which is less interfered by other nodes
void
AODV::compute_receivers_for_channels() {
	AODV_Neighbor *nb = nbhead.lh_first;
	AODV_Neighbor *nbn;
	int minimum=100000;

	bool already_seen[MAXIMUM_NODES];
 
	// Initialize the number of interferers for channel ...
	for (int i=0; i<MAX_CHANNELS; i++)
		num_recv_channels_[i]=0;
  
	for (int i=0; i< MAXIMUM_NODES; i++)
		already_seen[i]=false;

	for(; nb; nb = nbn) {
   
	   // We should  count the same receiver only once ...
 	  	if (already_seen[nb->nb_addr] == false) {
 			  num_recv_channels_[nb->channel]++;
			  already_seen[nb->nb_addr]=true;
   		}
  
   		nbn = nb->nb_link.le_next;
	}
#ifdef LI_MOD
	(void)minimum; // Just to avoid warning - Li
#endif

}



//get_less_interfered_channel: Return the channel with the lowest number of nodes tx on it
int 
AODV::get_less_interfered_channel() {
	
	int minimum=100000;
	int channel_id;

	for (int i=0; i<MAX_CHANNELS; i++) {
		if (num_recv_channels_[i] < minimum) {
			minimum=num_recv_channels_[i];
			channel_id=i;
		}
	}
	
	return channel_id;
}



//recvNotification: receive a notification about the detection of an active PU on the rx channel
//Spectrum Decision/mobility has been performed at MAC Layer
//The CR informs the neighbouring nodes about the selection of the new channel
void
AODV::recvNotification(Packet *p) {

#ifndef LI_MOD // No LI_MOD
	// Notify the occurrence of a spectrum Handoff to the neighbouring nodes
	sendHello(); 
#else // LI_MOD
	printf("\n [PU Detection Notification!] Node %d receive MAC layer's notification at %f \n\n", 
			index, CURRENT_TIME);

	struct hdr_cmn* ch = HDR_CMN(p);
	aodv_rt_entry *rt, *rtn;

	Packet *rerr_1 = Packet::alloc();
	Packet *rerr_2 = Packet::alloc();
	Packet *rerr_3 = Packet::alloc();
	struct hdr_aodv_error *re_1 = HDR_AODV_ERROR(rerr_1);
	struct hdr_aodv_error *re_2 = HDR_AODV_ERROR(rerr_2);
	struct hdr_aodv_error *re_3 = HDR_AODV_ERROR(rerr_3);
	re_1->DestCount = 0;
	re_2->DestCount = 0;
	re_3->DestCount = 0;

	for(int i=0; i < ch->udst_num_; i++) { // for each udst

		bool dst_ = true;
		int id_ = ch->udst[i];

		for(rt = rtable.head(); rt; rt = rtn) {  // for each rt entry
			rtn = rt->rt_link.le_next;
			if ((rt->rt_hops != INFINITY2) && (rt->rt_dst == id_) ) {  // check the dst not next hop 
				assert (rt->rt_flags == RTF_UP);
				assert((rt->rt_seqno%2) == 0);
				// If we can find this routing entry, then this node is not the dst for this flow - Li
				dst_ = false;
				rt->rt_seqno++;
				re_1->unreachable_dst_seqno[re_1->DestCount] = rt->rt_seqno;
				re_2->unreachable_dst_seqno[re_2->DestCount] = rt->rt_seqno;
				re_3->unreachable_dst_seqno[re_3->DestCount] = rt->rt_seqno;
				#ifdef DEBUG
				fprintf(stderr, "%s(%f): %d\t(%d\t%u\t%d)\n", __FUNCTION__, CURRENT_TIME,
						index, re->unreachable_dst[re->DestCount],
						re->unreachable_dst_seqno[re->DestCount], rt->rt_nexthop);
				#endif // DEBUG
				rt_down(rt);
			}
			// remove the lost neighbor from all the precursor lists
			rt->pc_delete(id_);
		} 

		if(dst_) {
				// If this node is a dst, then it can not find a routing entry
				re_1->unreachable_dst_seqno[re_1->DestCount] = seqno; // use local seqno
				re_2->unreachable_dst_seqno[re_2->DestCount] = seqno;
				re_3->unreachable_dst_seqno[re_3->DestCount] = seqno;
		}

		re_1->unreachable_dst[re_1->DestCount] = id_;
		re_1->DestCount += 1;
		re_2->unreachable_dst[re_2->DestCount] = id_;
		re_2->DestCount += 1;
		re_3->unreachable_dst[re_3->DestCount] = id_;
		re_3->DestCount += 1;
	}

	sendError(rerr_1, false); // be careful about who should handle link failure - Li
	sendError(rerr_2, true); // we use random delay for the second Error Packet 
	sendError(rerr_3, true); // we use random delay for the third Error Packet 

	seqno += 2;     // Set of neighbors changed
	assert ((seqno%2) == 0);
#endif // LI_MOD

	Packet::free(p);
}

 // CRAHNs Model END
 

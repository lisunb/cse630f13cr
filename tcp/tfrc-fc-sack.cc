/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1999  International Computer Science Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by ACIRI, the AT&T 
 *      Center for Internet Research at ICSI (the International Computer
 *      Science Institute).
 * 4. Neither the name of ACIRI nor of ICSI may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ICSI AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL ICSI OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdlib.h>
#include <sys/types.h>
#include <math.h>
 
#include "tfrc-fc-sack.h"
#include "formula.h"
#include "flags.h"

int hdr_tfrc_fc::offset_;
int hdr_tfrc_fc_fb::offset_;
int hdr_tfrc_fc_sack::offset_;

static class TFRC_FCHeaderClass : public PacketHeaderClass {
public:
	TFRC_FCHeaderClass() : PacketHeaderClass("PacketHeader/TFRC_FC",
					      sizeof(hdr_tfrc_fc)) {
		bind_offset(&hdr_tfrc_fc::offset_);
	}
} class_tfrc_fchdr;

static class TFRC_FC_FBHeaderClass : public PacketHeaderClass {
public:
	TFRC_FC_FBHeaderClass() : PacketHeaderClass("PacketHeader/TFRC_FC_FB",
						  sizeof(hdr_tfrc_fc_fb)) {
		bind_offset(&hdr_tfrc_fc_fb::offset_);
	}
} class_tfrc_fc_fbhdr;

static class TFRC_FC_SACKHeaderClass : public PacketHeaderClass {
public:
	TFRC_FC_SACKHeaderClass() : PacketHeaderClass("PacketHeader/TFRC_FC_SACK",
						  sizeof(hdr_tfrc_fc_sack)) {
		bind_offset(&hdr_tfrc_fc_sack::offset_);
	}
} class_tfrc_fc_sackhdr;

static class TfrcFcClass : public TclClass {
public:
  	TfrcFcClass() : TclClass("Agent/TFRC_FC") {}
  	TclObject* create(int, const char*const*) {
    		return (new TfrcFcAgent());
  	}
} class_tfrc_fc;


TfrcFcAgent::TfrcFcAgent() : Agent(PT_TFRC_FC), send_timer_(this), 
	 NoFeedbacktimer_(this), retransmission_timer_(this), rate_(0), oldrate_(0), maxrate_(0)
{
	bind("packetSize_", &size_);
	bind("rate_", &rate_);
	bind("df_", &df_);
	bind("tcp_tick_", &tcp_tick_);
	bind("ndatapack_", &ndatapack_);
	bind("ndatabytes_", &ndatabytes_);
	bind("true_loss_rate_", &true_loss_rate_);
	bind("srtt_init_", &srtt_init_);
	bind("rttvar_init_", &rttvar_init_);
	bind("rtxcur_init_", &rtxcur_init_);
	bind("rttvar_exp_", &rttvar_exp_);
	bind("T_SRTT_BITS", &T_SRTT_BITS);
	bind("T_RTTVAR_BITS", &T_RTTVAR_BITS);
	bind("InitRate_", &InitRate_);
	bind("overhead_", &overhead_);
	bind("ssmult_", &ssmult_);
	bind("bval_", &bval_);
	bind("ca_", &ca_);
	bind_bool("printStatus_", &printStatus_);
	bind_bool("conservative_", &conservative_);
	bind_bool("ecn_", &ecn_);
	bind("minrto_", &minrto_);
	bind("maxHeavyRounds_", &maxHeavyRounds_);
	bind("SndrType_", &SndrType_); 
	bind("scmult_", &scmult_);
	bind_bool("oldCode_", &oldCode_);
	bind("rate_init_", &rate_init_);
	bind("rate_init_option_", &rate_init_option_);
	bind_bool("slow_increase_", &slow_increase_); 
	bind_bool("voip_", &voip_);
	bind("voip_max_pkt_rate_", &voip_max_pkt_rate_);
	bind("fsize_", &fsize_);
	bind_bool("useHeaders_", &useHeaders_);
	bind_bool("idleFix_", &idleFix_);
	bind("headersize_", &headersize_);
	seqno_ = -1;
	aid_ = -1;
	maxseq_ = 0;
	datalimited_ = 0;
	lastlimited_ = 0.0;
	last_pkt_time_ = 0.0;
	bind("maxqueue_", &maxqueue_);
	maxqueue_ = MAXSEQ;
}

/*
 * Must convert bytes into packets. 
 * If nbytes == -1, this corresponds to infinite send.  We approximate
 * infinite by a very large number (MAXSEQ).
 * For simplicity, when bytes are converted to packets, fractional packets 
 * are always rounded up.  
 */
void TfrcFcAgent::sendmsg(int nbytes, const char* /*flags*/)
{
        if (nbytes == -1 && maxseq_ < MAXSEQ)
		advanceby(MAXSEQ - maxseq_);
        else if (size_ > 0) {
		int npkts = int(nbytes/size_);
		npkts += (nbytes%size_ ? 1 : 0);
		// maxqueue was added by Tom Phelan, to control the
		//   transmit queue from the application.
		if ((maxseq_ - seqno_) < maxqueue_) {
			advanceby(npkts);
		}
	}
}


void TfrcFcAgent::advanceby(int delta)
{
  	maxseq_ += delta;
	
	if (seqno_ == -1) {
		// if no packets hve been sent so far, call start. 
  		start(); 
	} else if (datalimited_ && maxseq_ > seqno_) {
		// We were data-limited - send a packet now!
		// The old code always waited for a timer to expire!!
		datalimited_ = 0;
		lastlimited_ = Scheduler::instance().clock();
		all_idle_ = 0;
		if (!oldCode_) {
			sendpkt();
		}
	}
} 

int TfrcFcAgent::command(int argc, const char*const* argv)
{
	if (argc==2) {
		// are we an infinite sender?
		if ( (strcmp(argv[1],"start")==0) && (SndrType_ == 0)) {
			start();
			return TCL_OK;
		}
		if (strcmp(argv[1],"stop")==0) {
			stop();
			return TCL_OK;
		}
	}
  	if ((argc == 3) && (SndrType_ == 1)) {
		// or do we need an FTP type app? 
    	if (strcmp(argv[1], "advance") == 0) {
            	int newseq = atoi(argv[2]);
		// THIS ISN"T USED.
		// newseq: new sequence
		// seqno_: next sequence to be sent
		// maxseq_: max seq_  produced by app so far.
            	if (newseq > maxseq_)
                 	advanceby(newseq - maxseq_);
            	return (TCL_OK);
    	}
    	if (strcmp(argv[1], "advanceby") == 0) {
      		advanceby(atoi(argv[2]));
      		return (TCL_OK);
    		}
	}
	return (Agent::command(argc, argv));
}

void TfrcFcAgent::start()
{
	seqno_=0;	
	rate_ = InitRate_;
	delta_ = 0;
	oldrate_ = rate_;  
	rate_change_ = SLOW_START;
	UrgentFlag = 1;
	rtt_=0;	 
	sqrtrtt_=1;
	rttcur_=1;
	tzero_ = 0;
	last_change_=0;
	maxrate_ = 0; 
	ss_maxrate_ = 0;
	ndatapack_=0;
	ndatabytes_ = 0;
	true_loss_rate_ = 0;
	active_ = 1; 
	round_id = 0;
	heavyrounds_ = 0;
	t_srtt_ = int(srtt_init_/tcp_tick_) << T_SRTT_BITS;
	t_rttvar_ = int(rttvar_init_/tcp_tick_) << T_RTTVAR_BITS;
	t_rtxcur_ = rtxcur_init_;
	rcvrate = 0 ;
	all_idle_ = 0;

	first_pkt_rcvd = 0 ;
	
	/* used for result FC SACK summary (by YL) */
#ifdef DEBUG_TRACEFCSACK	
	nb_rtx = 0;
	nb_fb = 0;
	nb_pkt_sent = 0;
	total_sackvec_length = 0;
	nb_rtx_no_fb = 0;
	nb_reduce_rate = 0;
	
	// trace send rate and free window of the sender
	FILE *pFile;
	pFile = fopen ("trace_rate_window.tr","w");
  	if (pFile!=NULL)
  	{
   		fprintf(pFile, "Time (sec) | Rate (kbps) | Free Window\n");
 		fclose (pFile);
  	}
  		// trace send rate and free window of the sender
	FILE *tFile;
	tFile = fopen ("trace_window_zero.tr","w");
  	if (tFile!=NULL)
  	{
   		fprintf(tFile, "Time (sec) | Rate (kbps) | Free Window\n");
 		fclose (tFile);
  	}
#endif  	
	
	// used by Flow Control (by YL)
	aid_ = 1;
	window_size = (int) MAXRCVBUFFERSIZE/size_;
	free_window_ = window_size;
	window_full_ = 0;
	
	// initialise sending buffer
	retransmission_required = 0;
	sackedsofar_ = -1;
	max_pkt_aid_ = -1;
	if((send_buffer_ = (sendbuf_element *)malloc(sizeof(sendbuf_element)*window_size)) != NULL) {
			for(int i = 0; i < window_size; i ++) {
				send_buffer_[i].timestamp = 0;
				send_buffer_[i].sacked = INIT;
				send_buffer_[i].aid = -1;
			}
	}
	else {
			printf ("error allocating memory for sending buffer\n");
			abort(); 
	}
	
	// send the first packet
	sendpkt();
	// ... at initial rate
	send_timer_.resched(size_/rate_);
	// ... and start timer so we can cut rate 
	// in half if we do not get feedback
	NoFeedbacktimer_.resched(2*size_/rate_); 
	// start retransmission timer so when no feedback
	// arrives then resend all unsacked packets in sending buffer
	retransmission_timer_.resched(4*size_/rate_);	
}

void TfrcFcAgent::stop()
{
#ifdef DEBUG_TRACEFCSACK	
	write_summary();
#endif	
	active_ = 0;
	if (idleFix_) 
 		datalimited_ = 1;
	send_timer_.force_cancel();
	//free((char *)send_buffer_);
}

/* send/retransmit data packet.
 * if SACK vector of FB msg indicated one loss, 
 * then one packet is marked as NEEDTORETRANSMIT and
 * retransmission_required is 1.
 * So first priority to retransmit then send new packets,
 * because retransmission is possible even when sender is blocked
 * from sending. (by YL)
 */
void TfrcFcAgent::nextpkt()
{
	double next = -1;
	double xrate = -1; 

	if (SndrType_ == 0) {
		if(retransmission_required) retransmit_packets();
		else sendpkt();
	}
	else {
		if (maxseq_ > seqno_) {
			if(retransmission_required) retransmit_packets();
			else sendpkt();
		} else
			datalimited_ = 1;
			if (debug_) {
			 	double now = Scheduler::instance().clock();
				printf("Time: %5.2f Datalimited now.\n", now);
			}
	}
	
	// If slow_increase_ is set, then during slow start, we increase rate
	// slowly - by amount delta per packet 
	// SALLY
    //    double now = Scheduler::instance().clock(); //notused
	// SALLY
	if (slow_increase_ && round_id > 2 && (rate_change_ == SLOW_START) 
		       && (oldrate_+SMALLFLOAT< rate_)) {
		oldrate_ = oldrate_ + delta_;
		xrate = oldrate_;
	} else {
		if (ca_) {
			if (debug_) printf("SQRT: now: %5.2f factor: %5.2f\n", Scheduler::instance().clock(), sqrtrtt_/sqrt(rttcur_));
			xrate = rate_ * sqrtrtt_/sqrt(rttcur_);
		} else
			xrate = rate_;
	}
	if (xrate > SMALLFLOAT) {
		next = size_/xrate;
		if (voip_) {
	  	    	double min_interval = 1.0/voip_max_pkt_rate_;
		    	if (next < min_interval)
				next = min_interval;
		}
		//
		// randomize between next*(1 +/- woverhead_) 
		//
		next = next*(2*overhead_*Random::uniform()-overhead_+1);
		if (next > SMALLFLOAT)
			send_timer_.resched(next);
                else 
			send_timer_.resched(SMALLFLOAT);
	}
}

void TfrcFcAgent::update_rtt (double tao, double now) 
{
	/* the TCP update */
	t_rtt_ = int((now-tao) /tcp_tick_ + 0.5);
	if (t_rtt_==0) t_rtt_=1;
	if (t_srtt_ != 0) {
		register short rtt_delta;
		rtt_delta = t_rtt_ - (t_srtt_ >> T_SRTT_BITS);    

		if ((t_srtt_ += rtt_delta) <= 0)    
			t_srtt_ = 1;
		if (rtt_delta < 0)
			rtt_delta = -rtt_delta;

	  	rtt_delta -= (t_rttvar_ >> T_RTTVAR_BITS);
	  	if ((t_rttvar_ += rtt_delta) <= 0)  
			t_rttvar_ = 1;
	} else {
		t_srtt_ = t_rtt_ << T_SRTT_BITS;		
		t_rttvar_ = t_rtt_ << (T_RTTVAR_BITS-1);	
	}
	t_rtxcur_ = (((t_rttvar_ << (rttvar_exp_ + (T_SRTT_BITS - T_RTTVAR_BITS))) + t_srtt_)  >> T_SRTT_BITS ) * tcp_tick_;
	tzero_=t_rtxcur_;
 	if (tzero_ < minrto_) 
  		tzero_ = minrto_;

	/* fine grained RTT estimate for use in the equation */
	if (rtt_ > 0) {
		rtt_ = df_*rtt_ + ((1-df_)*(now - tao));
		sqrtrtt_ = df_*sqrtrtt_ + ((1-df_)*sqrt(now - tao));
	} else {
		rtt_ = now - tao;
		sqrtrtt_ = sqrt(now - tao);
	}
	rttcur_ = now - tao;
}

/*
 * Receive a status (TFRC+FC/SACK or FC/SACK only) report from the receiver.
 */
void TfrcFcAgent::recv(Packet *pkt, Handler *)
{
	int first_sackvec_aid, sackvec_length;
	int old_sackedsofar, i, adv_win;
	int loss_in_sackvec = 0;
	int* vector = NULL;
	int* sackvec = NULL;
	hdr_tfrc_fc_fb *nck = NULL;
	int nextpkt_called = 0;

	double now = Scheduler::instance().clock();
#ifdef DEBUG_TRACEFCSACK	
	nb_fb++;
#endif	
	/* Flow Control and SACK msg by YL */
	old_sackedsofar = sackedsofar_;
	hdr_tfrc_fc_sack *tfrc_sackh = hdr_tfrc_fc_sack::access(pkt);
	/* TFRC feedback msg is included */
	nck = hdr_tfrc_fc_fb::access(pkt);

	// copy from Flow control & SACK msg
	adv_win = tfrc_sackh->window;
	ack_ = tfrc_sackh->ack;
	sackvec_length = tfrc_sackh->sack_length;

#ifdef DEBUG_TRACEFCSACK
	total_sackvec_length += sackvec_length;
#endif

	/* Copy the sack vector */
	sackvec = (int *)malloc(sackvec_length*SIZE_INT);
	if(sackvec == NULL) {
		printf ("error allocating memory for SACK vector\n");
		abort();
	}

	vector = (int*) pkt->accessdata();

	for(int i = 0; i < sackvec_length; i++)
		sackvec[i] = *vector++;

	/* Sack vector starts from ack (type AID) if FIRST vector element is 1
	 * and if FIRST vector element is 0 the sack vector starts at ack+1 (type AID) */
	if(sackvec[0] == 1) 
		first_sackvec_aid = ack_;
	else
		first_sackvec_aid = ack_ + 1;

#ifdef DEBUG_FCSACK		
	double sack_ts = tfrc_sackh->timestamp;
	/* print content of received FC/SACK msg */
	printf("\n############## RCVD SACK ################\n");
	printf("# %f ts: %f ack: %d win: %d vec_len: %d\n", now, sack_ts, ack_, adv_win, sackvec_length);
	printf("# SACK: [ ");
	for(i = 0; i < sackvec_length; i++)
		printf("%d ", sackvec[i]);
	printf("]\n");
	printf("# Sendbuffer: [ ");
	for(i = 0; i < (max_pkt_aid_-sackedsofar_); i++)
		printf("%d ", send_buffer_[i].aid);
	printf("]\n###############################################\n\n");
#endif

	/* if a Feedback msg is lost, ACK is higher than sackedsofar_+1 
	 * -> acknowledge packets in sending buffer up to ACK */
	if(ack_ > sackedsofar_+1) {
#ifdef DEBUG_FCSACK
		printf("[TFRC FC] We lost a Feedback msg before! AID (%d) in sackvec is higher than sackedsofar+1 (%d). SACK up to AID\n", ack_, sackedsofar_+1);
#endif
//		for (i = 0; i<(first_sackvec_aid - sackedsofar_+1); i++) {
//			send_buffer_[i].sacked = SACKED;
//			send_buffer_[i].timestamp = now;
//		}
		slide_send_buffer((ack_-sackedsofar_+1), sackedsofar_);
		sackedsofar_= ack_-1;
		old_sackedsofar = sackedsofar_;
	}
	
	/* sacknowledge the sending buffer */
	if(sackvec_length > 0 && first_sackvec_aid == sackedsofar_+1) {
		
#ifdef DEBUG_FCSACK
		printf("[TFRC FC] Rcv: sackedsofar+1 (%d) EQUAL to first_sackvec_aid (%d) of SACK\n", sackedsofar_+1, first_sackvec_aid);
#endif		
		for(i = 0; i<sackvec_length; i++) {
			if (sackvec[i] == 1) {
				send_buffer_[i].sacked = SACKED;
				send_buffer_[i].timestamp = now;
				if(loss_in_sackvec == 0) {
					sackedsofar_++;
				}
			}
			else {
				loss_in_sackvec = 1;
#ifdef DEBUG_FCSACK				
				printf("[TFRC FC] Rcv: Loss of packet aid: %d\n", first_sackvec_aid+i);
#endif			
				// check if this packet was transmitted in the same rtt
				if((now-send_buffer_[i].timestamp) >= (0.5)*rtt_) {
					//retransmit(send_buffer_[i].aid);
					//retransmit(first_sackvec_aid+i);
					send_buffer_[i].sacked = NEEDTORETRANSMIT;
					retransmission_required = 1;
					//send_buffer_[i].timestamp = now;
				}					
			}
		}
		
		/* slide the window of sending buffer by the value of (sackedsofar - old_sackedsofar) */
		if(sackedsofar_ > old_sackedsofar) {
			slide_send_buffer((sackedsofar_-old_sackedsofar), old_sackedsofar);
			
			/* adapt free window at sender */
			free_window_ = adv_win - (max_pkt_aid_ - (old_sackedsofar + sackvec_length));
			if(free_window_ < 0) {
				printf("[TFRC FC] Rcv ERROR: Negative free_window (free_window_: %d, adv_win: %d, max_pkt_aid_: %d, old_sackedsofar: %d, sackvec_length: %d\n",
						free_window_, adv_win, max_pkt_aid_, old_sackedsofar, sackvec_length);
				abort();
			}
#ifdef DEBUG_FCSACK			
			printf("[TFRC FC] Rcv: New free window is: %d (adv_win: %d, max_pkt_aid: %d, old_sackedsofar: %d, sackvec_length: %d\n",
					 free_window_, adv_win, max_pkt_aid_, old_sackedsofar, sackvec_length);
#endif			
			if(retransmission_required)
				nextpkt();			
			else if(free_window_ > 0 && window_full_) {
			/* if sending buffer is again available, continue to send */					
				window_full_ = 0;
				nextpkt();
			}
		}
			
	}
	else {
		/* if the SACK msg is sent because receiver's buffer is full
		 * doesn't happen much. 
		 * (empty sack vector and win = 0)*/
		free_window_ = adv_win;
#ifdef DEBUG_FCSACK	
		if(first_sackvec_aid != sackedsofar_+1	) {	
			printf("[TFRC FC] RCV WARNING sackedsofar+1 (%d) NOT equal to first_sackvec_aid (%d) of SACK\n", sackedsofar_+1, first_sackvec_aid);
		}
#endif		
	}
	
	free(sackvec);

	
	// if we have unsacked packets in sending buffer retransmit them
	double rt = 4*rtt_ ; 
		if (rt < 4*size_/rate_) 
			rt = 4*size_/rate_ ; 
	retransmission_timer_.resched(rt);

	if(nck != NULL) {
		
		/* if floating point exception occurs, then use first expression instead of the one line below */
		//double ts = nck->timestamp_echo; 
		double ts = nck->timestamp_echo + nck->timestamp_offset;
	
		double rate_since_last_report = nck->rate_since_last_report;
		// double NumFeedback_ = nck->NumFeedback_;
		double flost = nck->flost; 
		int losses = nck->losses;
		true_loss_rate_ = nck->true_loss;
	
		round_id ++ ;
		UrgentFlag = 0;
	
		if (round_id > 1 && rate_since_last_report > 0) {
			/* compute the max rate for slow-start as two times rcv rate */ 
			ss_maxrate_ = 2*rate_since_last_report*size_;
			if (conservative_) { 
				if (losses >= 1) {
					/* there was a loss in the most recent RTT */
					if (debug_) printf("time: %5.2f losses: %d rate %5.2f\n", 
					  now, losses, rate_since_last_report);
					maxrate_ = rate_since_last_report*size_;
				} else { 
					/* there was no loss in the most recent RTT */
					maxrate_ = scmult_*rate_since_last_report*size_;
				}
				if (debug_) printf("time: %5.2f losses: %d rate %5.2f maxrate: %5.2f\n", now, losses, rate_since_last_report, maxrate_);
			} else 
				maxrate_ = 2*rate_since_last_report*size_;
		} else {
			ss_maxrate_ = 0;
			maxrate_ = 0; 
		}
			
		/* update the round trip time */
		update_rtt (ts, now);
	
		/* .. and estimate of fair rate */
		if (voip_ != 1) {
			// From RFC 3714:
			// The voip flow gets to send at the same rate as
			//  a TCP flow with 1460-byte packets.
			fsize_ = size_;
		}	
		rcvrate = p_to_b(flost, rtt_, tzero_, fsize_, bval_);
		// rcvrate is in bytes per second, based on fairness with a    
		// TCP connection with the same packet size size_.	    	  
		if (voip_) {
			// Subtract the bandwidth used by headers.
			double temp = rcvrate*(size_/(1.0*headersize_+size_));
			rcvrate = temp;
		}
	
		/* if we get no more feedback for some time, cut rate in half */
		double t = 2*rtt_ ; 
		if (t < 2*size_/rate_) 
			t = 2*size_/rate_ ; 
		NoFeedbacktimer_.resched(t);
		
		/* if we are in slow start and we just saw a loss */
		/* then come out of slow start */
	
		if (first_pkt_rcvd == 0) {
			first_pkt_rcvd = 1 ; 
			slowstart();
			nextpkt();
			nextpkt_called = 1;
		}
		else {
			if (rate_change_ == SLOW_START) {
				if (flost > 0) {
					rate_change_ = OUT_OF_SLOW_START;
					oldrate_ = rate_ = rcvrate;
				}
				else {
					slowstart();
					nextpkt();
					nextpkt_called = 1;
				}
			}
			else {
				if (rcvrate>rate_) 
					increase_rate(flost);
				else 
					decrease_rate ();		
			}
		}
		if (printStatus_) {
			printf("time: %5.2f rate: %5.2f\n", now, rate_);
			double packetrate = rate_ * rtt_ / size_;
			printf("time: %5.2f packetrate: %5.2f\n", now, packetrate);
		}
	}

//	/* if we get no more feedback for some time, cut rate in half */
//	double t = 2*rtt_ ; 
//	if (t < 2*size_/rate_) 
//		t = 2*size_/rate_ ; 
//	NoFeedbacktimer_.resched(t);
	
//	/* if sending buffer is full, but there are packets to retransmit, 
//	 * then retransmit immediately */
//	if(!nextpkt_called && send_timer_.status() == TIMER_IDLE 
//		&& retransmission_required)
//		nextpkt();
				
	Packet::free(pkt);
}

/*
 * Calculate initial sending rate from RFC 3390.
 */
double TfrcFcAgent::rfc3390(int size)
{
        if (size_ <= 1095) {
                return (4.0);
        } else if (size_ < 2190) {
                return (3.0);
        } else {
                return (2.0);
        }
}

/*
 * Used in setting the initial rate.
 * This is from TcpAgent::initial_wnd().
 */
double TfrcFcAgent::initial_rate()
{
        if (rate_init_option_ == 1) {
		// init_option = 1: static initial rate of rate_init_
                return (rate_init_);
        }
        else if (rate_init_option_ == 2) {
                // do initial rate according to RFC 3390.
		return (rfc3390(size_));
        }
        // XXX what should we return here???
        fprintf(stderr, "Wrong number of rate_init_option_ %d\n",
                rate_init_option_);
        abort();
        return (2.0); // XXX make msvc happy.
}


// ss_maxrate_ = 2*rate_since_last_report*size_;
// rate_: the rate set from the last pass through slowstart()
void TfrcFcAgent::slowstart () 
{
	double now = Scheduler::instance().clock(); 
	double initrate = initial_rate()*size_/rtt_;
	// If slow_increase_ is set to true, delta is used so that 
	//  the rate increases slowly to new value over an RTT. 
	if (debug_) printf("SlowStart: round_id: %d rate: %5.2f ss_maxrate_: %5.2f\n", round_id, rate_, ss_maxrate_);
	if (round_id <=1 || (round_id == 2 && initial_rate() > 1)) {
		// We don't have a good rate report yet, so keep to  
		//   the initial rate.				     
		oldrate_ = rate_;
		if (rate_ < initrate) rate_ = initrate;
		delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
		last_change_=now;
	} else if (ss_maxrate_ > 0) {
		if (idleFix_ && (datalimited_ || lastlimited_ > now - 1.5*rtt_)
			     && ss_maxrate_ < initrate) {
			// Datalimited recently, and maxrate is small.
			// Don't be limited by maxrate to less that initrate.
			oldrate_ = rate_;
			if (rate_ < initrate) rate_ = initrate;
			delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
			last_change_=now;
		} else if (rate_ < ss_maxrate_ && 
		                    now - last_change_ > rtt_) {
			// Not limited by maxrate, and time to increase.
			// Multiply the rate by ssmult_, if maxrate allows.
			oldrate_ = rate_;
			if (ssmult_*rate_ > ss_maxrate_) 
				rate_ = ss_maxrate_;
			else rate_ = ssmult_*rate_;
			if (rate_ < size_/rtt_) 
				rate_ = size_/rtt_; 
			delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
			last_change_=now;
		} else if (rate_ > ss_maxrate_) {
			// Limited by maxrate.  
			rate_ = oldrate_ = ss_maxrate_/2.0;
			delta_ = 0;
			last_change_=now;
		} 
	} else {
		// If we get here, ss_maxrate <= 0, so the receive rate is 0.
		// We should go back to a very small sending rate!!!
		oldrate_ = rate_;
		rate_ = size_/rtt_; 
		delta_ = 0;
        	last_change_=now;
	}
	if (debug_) printf("SlowStart: now: %5.2f rate: %5.2f delta: %5.2f\n", now, rate_, delta_);
}

void TfrcFcAgent::increase_rate (double p)
{               
    double now = Scheduler::instance().clock();
    double maximumrate;

	double mult = (now-last_change_)/rtt_ ;
	if (mult > 2) mult = 2 ;

	rate_ = rate_ + (size_/rtt_)*mult ;
	if (datalimited_ || lastlimited_ > now - 1.5*rtt_) {
		// Modified by Sally on 3/10/2006
		// If the sender has been datalimited, rate should be
		//   at least the initial rate, when increasing rate.
		double init_rate = initial_rate()*size_/rtt_;
	   	maximumrate = (maxrate_>init_rate)?maxrate_:init_rate ;
        } else {
	   	maximumrate = (maxrate_>size_/rtt_)?maxrate_:size_/rtt_ ;
	}
	maximumrate = (maximumrate>rcvrate)?rcvrate:maximumrate;
	rate_ = (rate_ > maximumrate)?maximumrate:rate_ ;
	
        rate_change_ = CONG_AVOID;  
        last_change_ = now;
	heavyrounds_ = 0;
	if (debug_) printf("Increase: now: %5.2f rate: %5.2f lastlimited: %5.2f rtt: %5.2f\n", now, rate_, lastlimited_, rtt_);
}       

void TfrcFcAgent::decrease_rate () 
{
	double now = Scheduler::instance().clock(); 
	rate_ = rcvrate;
	double maximumrate = (maxrate_>size_/rtt_)?maxrate_:size_/rtt_ ;

	// Allow sending rate to be greater than maximumrate
	//   (which is by default twice the receiving rate)
	//   for at most maxHeavyRounds_ rounds.
	if (rate_ > maximumrate)
		heavyrounds_++;
	else
		heavyrounds_ = 0;
	if (heavyrounds_ > maxHeavyRounds_) {
		rate_ = (rate_ > maximumrate)?maximumrate:rate_ ;
	}

	rate_change_ = RATE_DECREASE;
	last_change_ = now;
}

void TfrcFcAgent::sendpkt()
{
	// sending buffer is full -> wait until we receive non-zero window advertisement
	if(free_window_ <= 0) {
#ifdef DEBUG_TRACEFCSACK		
		trace_window_zero();
#endif
		
#ifdef DEBUG_FCSACK	
		printf("[TFRC FC] Send: Try to send pkt with seqno %d (aid %d), but sending window is full: %d -> freeze\n", seqno_, aid_, free_window_);
#endif		
		window_full_ = 1;
		send_timer_.force_cancel();
	}
	else if (active_) {
#ifdef DEBUG_TRACEFCSACK	
	trace_rate_window();
#endif
		double now = Scheduler::instance().clock(); 
		Packet* p = allocpkt();
		hdr_tfrc_fc *tfrch = hdr_tfrc_fc::access(p);
		hdr_flags* hf = hdr_flags::access(p);
		if (ecn_) {
			hf->ect() = 1;  // ECN-capable transport
		}
		tfrch->seqno=seqno_++;
		int aid=aid_++;
		tfrch->aid=aid;
		tfrch->timestamp=Scheduler::instance().clock();
		tfrch->rtt=rtt_;
		tfrch->tzero=tzero_;
		tfrch->rate=rate_;
		tfrch->psize=size_;
		tfrch->fsize=fsize_;
		tfrch->UrgentFlag=UrgentFlag;
		tfrch->round_id=round_id;
		ndatapack_++;
		ndatabytes_ += size_;
		if (useHeaders_ == true) {
			hdr_cmn::access(p)->size() += headersize_;
		}
		last_pkt_time_ = now;
		
		if(sackedsofar_ < 0) {
			// sending the first packet by YL
			sackedsofar_ = aid-1;
			max_pkt_aid_ = aid;
		}
		
		/* append packet to send_buffer_ */
		if(sendbuffer_append(p, aid) != 1) {
			printf("[TFRC FC] Send: ERROR: sendbuffer append \n");
			abort();
		}
#ifdef DEBUG_TRACEFCSACK		
		nb_pkt_sent++;
#endif		
		send(p, 0);
	}
}

/* append Packet to sendbuffer (=data waiting queue) */
int TfrcFcAgent::sendbuffer_append(Packet* p, int aid)
{
	double now = Scheduler::instance().clock();
	
	if(aid <= max_pkt_aid_ && max_pkt_aid_ != 1) {
		printf("[TFRC FC] ERROR!!! sendbuffer_append: new aid (%d) is NOT higher than max_pkt_aid (%d)\n", aid, max_pkt_aid_);
		return 0;
	}		
	//Packet* pkt_tmp = allocpkt();

	send_buffer_[(aid-sackedsofar_)-1].timestamp = now;
	send_buffer_[(aid-sackedsofar_)-1].aid = aid;
	send_buffer_[(aid-sackedsofar_)-1].sacked = WAITINGSACK;

	max_pkt_aid_ = aid;
	free_window_--;

#ifdef DEBUG_FCSACK
	if(free_window_ < 0) {
		printf("[TFRC sendbuffer_append] ERROR: Negative free_window\n");
		return 0;
	}
	
	// print content of queued packet
	printf("################ sent/queued Packet ################\n");
	printf("# time: %f aid: %d win: %d\n", send_buffer_[(aid-sackedsofar_)-1].timestamp, send_buffer_[(aid-sackedsofar_)-1].aid, free_window_+1);
	printf("####################################################\n");
#endif
	
	return 1;
}

/* "slide" sending buffer */
void TfrcFcAgent::slide_send_buffer(int length_to_delete, int old_sackedsofar)
{
	int i;
	sendbuf_element *send_buffer_temp = NULL;
	
	if((send_buffer_temp = (sendbuf_element *)malloc(sizeof(sendbuf_element)*window_size)) == NULL) {
		printf ("error allocating memory for temp sending buffer\n");
		abort();
	}

	for (i = 0; i < ((max_pkt_aid_-old_sackedsofar)-length_to_delete); i++) {
		send_buffer_temp[i].timestamp = send_buffer_[i+length_to_delete].timestamp;
		send_buffer_temp[i].sacked = send_buffer_[i+length_to_delete].sacked;
		send_buffer_temp[i].aid = send_buffer_[i+length_to_delete].aid;
	}
	
	for (i = ((max_pkt_aid_-old_sackedsofar)-length_to_delete); i < window_size; i++) {
		send_buffer_temp[i].timestamp = 0;
		send_buffer_temp[i].sacked = INIT;
		send_buffer_temp[i].aid = INIT;
	}
	
	send_buffer_ = send_buffer_temp;
}

/* RTX when no feedback*/
void TfrcFcAgent::retransmit_after_no_feedback()
{
	double now = Scheduler::instance().clock();
	 
	if(max_pkt_aid_-sackedsofar_ > 0) {
		// there is some unsacked packets in sending buffer -> retransmit them all
		for (int i = 0; i < max_pkt_aid_-sackedsofar_; i++) {
			if(send_buffer_[i].sacked != SACKED) {
#ifdef DEBUG_TRACEFCSACK				
				nb_rtx_no_fb++;
#endif				
				if((now-send_buffer_[i].timestamp) >= (0.5)*rtt_) {
					send_buffer_[i].sacked = NEEDTORETRANSMIT;
					retransmission_required = 1;
				}			
			}
		}
		double rt = 4*rtt_ ; 
		if (rt < 4*size_/rate_) 
			rt = 4*size_/rate_ ; 
		retransmission_timer_.resched(rt);	
	}
	else {
		// no packets in sending buffer
		// assume there is no more data to send
		// stop retransmission_timer
		retransmission_timer_.force_cancel();
	}	
}

// retransmit all packets in sending buffer marked as NEEDTORETRANSMIT
void TfrcFcAgent::retransmit_packets()
{
	double now = Scheduler::instance().clock();
	int i = 0;
	int sent_one = 0;
	int still_pkt_retransmit = 0;
	
	if(max_pkt_aid_-sackedsofar_ > 0) {
		// retransmit all packets marked as NEEDTORETRANSMIT
		while(!sent_one && (i < max_pkt_aid_-sackedsofar_)) {
			if(send_buffer_[i].sacked == NEEDTORETRANSMIT) {
				retransmit(send_buffer_[i].aid);
				send_buffer_[i].timestamp = now;
				send_buffer_[i].sacked = RETRANSMITTED;
				sent_one = 1;	
			}
			i++;
		}
		//check if the retransmitted packet was the last retransmission required pkt...
		for(i = 0; i < max_pkt_aid_-sackedsofar_; i++) {
			if(send_buffer_[i].sacked == NEEDTORETRANSMIT)
				still_pkt_retransmit = 1;
		}
		if(!still_pkt_retransmit) retransmission_required = 0;
	}
}

void TfrcFcAgent::retransmit(int aid)
{
#ifdef DEBUG_FCSACK				
	printf("[TFRC FC] Rcv: Retransmission of packet aid: %d\n", aid);
#endif	
	
#ifdef DEBUG_TRACEFCSACK	
	nb_rtx++;
	trace_rate_window();
#endif

	/* Create new packet for retransmission */
	double now = Scheduler::instance().clock(); 
	Packet* p = allocpkt();
	hdr_tfrc_fc *tfrch = hdr_tfrc_fc::access(p);
	hdr_flags* hf = hdr_flags::access(p);
	if (ecn_) {
		hf->ect() = 1;  // ECN-capable transport
	}
	tfrch->seqno=seqno_++;
	tfrch->aid=aid;
	tfrch->timestamp=Scheduler::instance().clock();
	tfrch->rtt=rtt_;
	tfrch->tzero=tzero_;
	tfrch->rate=rate_;
	tfrch->psize=psize_;
	tfrch->fsize=fsize_;
	tfrch->UrgentFlag=UrgentFlag;
	tfrch->round_id=round_id;
	// Do we need to trace retransmitted packets? by YL
	ndatapack_++;	
	ndatabytes_ += size_;
	
	if (useHeaders_ == true) {
		hdr_cmn::access(p)->size() += headersize_;
	}
	last_pkt_time_ = now;
	
#ifdef DEBUG_FCSACK		
	printf("[TFRC FC] Retransmit: Queued pkt (aid: %d) RETRANSMITTED\n", aid);
#endif
	
	send(p, 0);	
}


/*
 * RFC 3448:
 * "If the sender has been idle since this nofeedback timer was set and
 * X_recv is less than four packets per round-trip time, then X_recv
 * should not be halved in response to the timer expiration.  This
 * ensures that the allowed sending rate is never reduced to less than
 * two packets per round-trip time as a result of an idle period."
 */
 
void TfrcFcAgent::reduce_rate_on_no_feedback()
{
#ifdef DEBUG_TRACEFCSACK
	nb_reduce_rate++;
#endif
	double now = Scheduler::instance().clock();
	rate_change_ = RATE_DECREASE; 
	if (oldCode_ || (!all_idle_ && !datalimited_)) {
		// if we are not datalimited
		rate_*=0.5;
	} else if ((datalimited_ || all_idle_) && rate_init_option_ == 1) { 
		// all_idle_: the sender has been datalimited since the 
		//    timer was set
		//  Don't reduce rate below rate_init_ * size_/rtt_.
                if (rate_ > 2.0 * rate_init_ * size_/rtt_ ) {
                        rate_*=0.5;
                } 
	} else if ((datalimited_ || all_idle_) && rate_init_option_ == 2) {
                // Don't reduce rate below the RFC3390 rate.
                if (rate_ > 2.0 * rfc3390(size_) * size_/rtt_ ) {
                        rate_*=0.5;
                } else if ( rate_ > rfc3390(size_) * size_/rtt_ ) {
                        rate_ = rfc3390(size_) * size_/rtt_;
                }
        }
	if (debug_) printf("NO FEEDBACK: time: %5.2f rate: %5.2f all_idle: %d\n", now, rate_, all_idle_);
	UrgentFlag = 1;
	round_id ++ ;
	double t = 2*rtt_ ; 
	// Set the nofeedback timer.
	if (t < 2*size_/rate_) 
		t = 2*size_/rate_ ; 
	NoFeedbacktimer_.resched(t);
	if (datalimited_) {
		all_idle_ = 1;
		if (debug_) printf("Time: %5.2f Datalimited now.\n", now);
	}
	nextpkt();
}

void TfrcFcSendTimer::expire(Event *) {
  	a_->nextpkt();
}

void TfrcFcNoFeedbackTimer::expire(Event *) {
	a_->reduce_rate_on_no_feedback ();
	// TODO: if the first (SYN) packet was dropped, then don't use
	//   the larger initial rates from RFC 3390:
        // if (highest_ack_ == -1 && wnd_init_option_ == 2)
	//     wnd_init_option_ = 1;
}
void TfrcFcRetransmissionTimer::expire(Event *) {
	/* FC SACK: Retransmit all unsacked packets in the sending buffer */
	a_->retransmit_after_no_feedback();
}

#ifdef DEBUG_TRACEFCSACK
void TfrcFcAgent::write_summary() 
{
	FILE * pFile;
	
	pFile = fopen ("tfrc_fc_summary.txt","a");
  	if (pFile!=NULL)
  	{
  		fprintf(pFile, "\n TFRC FC SACK summery:\n");
  		fprintf(pFile, "-----------------------\n");
  		fprintf(pFile, "Nb of pkt sent: %d\n", nb_pkt_sent);
  		fprintf(pFile, "Nb of retransmission: %d\n", nb_rtx);
  		fprintf(pFile, "Nb of reducing rate due to no feedback: %d\n", nb_reduce_rate);
  		fprintf(pFile, "Nb of retransmitted packet due to no feedback: %d\n", nb_rtx_no_fb);  		
  		fprintf(pFile, "Nb of feedback msg received: %d\n", nb_fb);
  		fprintf(pFile, "Avg length of SACK vector: %d\n", (int)((double)total_sackvec_length/(double)nb_fb));  		
		fclose (pFile);
  	}	
}

void TfrcFcAgent::trace_rate_window()
{
	FILE * pFile;
	double now = Scheduler::instance().clock();
	
	pFile = fopen ("trace_rate_window.tr","a");
  	if (pFile!=NULL)
  	{
   		fprintf(pFile, "%f %f %d\n", now, ((double)8/(double)1024)*rate_, free_window_*10);
 		fclose (pFile);
  	}	
		
}

void TfrcFcAgent::trace_window_zero()
{
	FILE * tFile;
	double now = Scheduler::instance().clock();
	
	tFile = fopen ("trace_window_zero.tr","a");
  	if (tFile!=NULL)
  	{
   		fprintf(tFile, "%f %f\n", now, ((double)8/(double)1024)*rate_);
 		fclose (tFile);
  	}	
		
}
#endif







	

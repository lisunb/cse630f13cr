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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <math.h>
 
#include "tfrc-fc-sack-sink.h"
#include "flags.h"

static class TfrcFcSinkClass : public TclClass {
public:
  	TfrcFcSinkClass() : TclClass("Agent/TFRC_FCSink") {}
  	TclObject* create(int, const char*const*) {
     		return (new TfrcFcSinkAgent());
  	}
} class_tfrc_fcSink; 

TfrcFcSinkAgent::TfrcFcSinkAgent() : Agent(PT_TFRC_FC_FB), delivertimer_(this), nack_timer_(this) 
{
	bind("packetSize_", &size_);	
	bind("InitHistorySize_", &hsz);
	bind ("AdjustHistoryAfterSS_", &adjust_history_after_ss);
	bind ("printLoss_", &printLoss_);
	bind ("algo_", &algo); // algo for loss estimation
	bind ("PreciseLoss_", &PreciseLoss_);
	bind ("numPkts_", &numPkts_);

	// for WALI ONLY
	bind ("NumSamples_", &numsamples);
	bind ("discount_", &discount);
	bind ("smooth_", &smooth_);
	bind ("ShortIntervals_", &ShortIntervals_);

	// EWMA use only
	bind ("history_", &history); // EWMA history

	// for RBPH use only
	bind("minlc_", &minlc); 

	bind("bytes_", &bytes_);
	rtt_ =  0; 
	tzero_ = 0;
	last_timestamp_ = 0;
	last_arrival_ = 0;
	last_report_sent=0;
	total_received_ = 0;
	total_losses_ = 0;
	total_dropped_ = 0;

	maxseq = -1;
	maxseqList = -1;
	rcvd_since_last_report  = 0;
	losses_since_last_report = 0;
	loss_seen_yet = 0;
	lastloss = 0;
	lastloss_round_id = -1 ;
	numPktsSoFar_ = 0;

	rtvec_ = NULL;
	tsvec_ = NULL;
	lossvec_ = NULL;

	// used by FC (by YL)
	//NumFeedback_ = 2;
	bind("NumFeedback_", &NumFeedback_);
	bind("applReadRate_", &applreadrate);
	rcv_buffer_full_state = false;

	// used by SACK (by YL)
	max_aid_ = -1;
	sackvec_length_ = 0;
	lastaidSACKsent_ = 0;
	deliveredtoapp_ = 0;

	// used by WALI and EWMA
	last_sample = 0;

	// used only for WALI 
	false_sample = 0;
	sample = NULL ; 
	weights = NULL ;
	mult = NULL ;
        losses = NULL ;
	count_losses = NULL ;
	sample_count = 1 ;
	mult_factor_ = 1.0;
	init_WALI_flag = 0;

	// used only for EWMA
	avg_loss_int = -1 ;
	loss_int = 0 ;

	// used only bu RBPH
	sendrate = 0 ; // current send rate
}

/*
 * This is a new loss event if it is at least an RTT after the beginning
 *   of the last one.
 * If PreciseLoss_ is set, the new_loss also checks that there is a
 *     new round_id.
 * The sender updates the round_id when it receives a new report from
 *   the receiver, and when it reduces its rate after no feedback.
 * Sometimes the rtt estimates can be less than the actual RTT, and
 *   the round_id will catch this.  This can be useful if the actual
 *   RTT increases dramatically.
 */
int TfrcFcSinkAgent::new_loss(int i, double tstamp)
{
	double time_since_last_loss_interval = tsvec_[i%hsz]-lastloss;
	if ((time_since_last_loss_interval > rtt_)
	     && (PreciseLoss_ == 0 || (round_id > lastloss_round_id))) {
		lastloss = tstamp;
		lastloss_round_id = round_id ;
                if (time_since_last_loss_interval < 2.0 * rtt_ &&
				algo == WALI) {
                        count_losses[0] = 1;
                }
		return TRUE;
	} else return FALSE;
}

double TfrcFcSinkAgent::estimate_tstamp(int before, int after, int i)
{
	double delta = (tsvec_[after%hsz]-tsvec_[before%hsz])/(after-before) ; 
	double tstamp = tsvec_[before%hsz]+(i-before)*delta ;
	return tstamp;
}

/*
 * Receive new data packet.  If appropriate, generate a new report.
 */
void TfrcFcSinkAgent::recv(Packet *pkt, Handler *)
{
	hdr_tfrc_fc *tfrch = hdr_tfrc_fc::access(pkt); 
	hdr_flags* hf = hdr_flags::access(pkt);
	double now = Scheduler::instance().clock();
	double p = -1;
	int ecnEvent = 0;
	int congestionEvent = 0;
	int UrgentFlag = 0;	// send loss report immediately
	int newdata = 0;	// a new data packet received
	bool deliver2app = false;
	int buffer_is_full = 0;
	FILE * pFile = NULL;

	if (algo == WALI && !init_WALI_flag) {
		init_WALI () ;
	}
	rcvd_since_last_report ++;
	total_received_ ++;
	// bytes_ was added by Tom Phelan, for reporting bytes received.
	bytes_ += hdr_cmn::access(pkt)->size();
	
#ifdef DEBUG_FCSACK	
	printf("[TFRC FC SINK] Packet with AID %d received\n", tfrch->aid);
#endif
	if (maxseq < 0) {
		// This is the first data packet.
		newdata = 1;
		maxseq = tfrch->seqno - 1 ;
		maxseqList = tfrch->seqno;
		rtvec_=(double *)malloc(sizeof(double)*hsz);
		tsvec_=(double *)malloc(sizeof(double)*hsz);
		lossvec_=(char *)malloc(sizeof(double)*hsz);
		if (rtvec_ && lossvec_) {
			int i;
			for (i = 0; i < hsz ; i ++) {
				lossvec_[i] = UNKNOWN;
				rtvec_[i] = -1; 
				tsvec_[i] = -1; 
			}
		}
		else {
			printf ("error allocating memory for packet buffers\n");
			abort (); 
		}
#ifdef DEBUG_TRACEFCSACK		
		// trace receiver's buffer occupancy
		pFile = fopen ("buffer_occupancy_trace.txt","w");
		if(pFile != NULL) {
			fprintf(pFile, "#Receiver's buffer occupancy (time, occupancy)\n");
			fclose(pFile);
		}
		else printf("Could not open/write buffer occupancy trace file\n");
		
		// trace receiver's buffer occupancy
		pFile = fopen ("trace_rcv_buffer_full_state.tr","w");
		if(pFile != NULL) {
			fprintf(pFile, "#Number of Buffer_Full state at Receiver\n");
			fclose(pFile);
		}
		else printf("Could not open/write buffer_full state trace file\n");
#endif

		/* initiate flowcontrol and SACK parameters by YL*/
		if(tfrch->psize == 0) 
			packetsize = DEFAULT_PACKETSIZE;
		else			
			packetsize = tfrch->psize;

		if(applreadrate == 0)
			applreadtimewindow = -1;
		else applreadtimewindow = ((double)(packetsize*8))/((double)(1024*applreadrate));
#ifdef DEBUG_FCSACK				
		printf("[TFRC FC SINK] Application's buffer read rate is %d kbps. (applreadtimewindow is %f)\n", applreadrate, applreadtimewindow);
#endif		
		deliver2app = true;	// true = the first packet should be delivered immediately
		max_aid_ = tfrch->aid - 1;	// AID starts always with 1
		sackvec_length_ = 0;
		lastaidSACKsent_ = 0;	// AID starts always at 1
		sackedsofar_ = lastaidSACKsent_;
		window_size = (int) MAXRCVBUFFERSIZE/packetsize;
			
#ifdef DEBUG_FCSACK				
		printf("[TFRC FC SINK] Initial: window_size: %d (MAXRCVBUFFERSIZE: %d, size_: %d)\n", window_size, MAXRCVBUFFERSIZE, packetsize);
#endif		
		free_window_ = window_size;
		if((sackvec_=(int *)malloc(SIZE_INT*window_size)) != NULL) {
			for(int i = 0; i < window_size; i ++) {
				sackvec_[i] = UNKNOWN;
			}
		}
		else {
			printf ("error allocating memory for SACK vector\n");
			abort(); 
		}
	}
	
	int seqno = tfrch->seqno;

	/* FC SACK treatment (including out of order and duplicate packets) by YL */
	int aid = tfrch->aid;
	
	if(!(hf->ect() == 1 && hf->ce() == 1) && (aid-lastaidSACKsent_) <= window_size) {

		// check if it's a duplicate
		if(aid <= lastaidSACKsent_ || sackvec_[(aid-lastaidSACKsent_)-1] == RCVD) {
			// duplicate
#ifdef DEBUG_FCSACK							
			printf("[TFRC FC SINK] Rcv: Duplicate packet received (aid: %d)-> DO SOMETHING?\n", aid);
#endif			
		}
		else {
			sackvec_[(aid-lastaidSACKsent_)-1] = RCVD;
			
			// delayed or retransmitted packet (filling gap)
			if(aid < max_aid_) {	/* is same as: max_aid_ = (lastaidSACKsent_ + sackvec_length_) */
#ifdef DEBUG_FCSACK								
				printf("[TFRC FC SINK] Rcv: Delayed/retransmitted packet received\n");
#endif
				// first element of sackvec is RCVD -> update sackedsofar_
				if(sackvec_[0] != UNKNOWN) {
					
					// update sackedsofar_: look for RCVD followed by UNKNOWN then seqno with RCVD is sackedsofar_
					int i = 0;
					bool found = false;
					while(i < (sackvec_length_-1) && !found) {
						if(sackvec_[i] == RCVD && sackvec_[i+1] == UNKNOWN) {
							sackedsofar_ = lastaidSACKsent_ + i + 1;
							found = true;
						}
						i++;
					}
					// check also the last two space of buffer.
					// if no RCVD followed by UNKNOWN found, maybe the buffer is full without any gaps
					if(!found && sackvec_[(sackvec_length_-1)] == RCVD && sackvec_[(sackvec_length_-2)] == RCVD)
						sackedsofar_ = lastaidSACKsent_ + sackvec_length_ ;
						
				}
			}
			
			if((aid-sackedsofar_) == 1) {
				// correct next packet
				sackedsofar_ = aid;
			}
			
			// update free_window variable if received packet is new one and not retransmitted or delayed one
			int new_sackvec_length = aid-lastaidSACKsent_;
			if(new_sackvec_length > sackvec_length_) {
				free_window_ -= (new_sackvec_length-sackvec_length_);
				sackvec_length_ = new_sackvec_length;
					
#ifdef DEBUG_FCSACK										
				printf("[TFRC FC SINK] Rcv: Updated free_window: %d\n", free_window_);
#endif				
			}
		}
		
		// this check can be removed - START
		if(free_window_ == 0) {
			// receiver buffer is full -> send SACK only immediately (buffer_is_full = 1)!
			rcv_buffer_full_state = true;
			buffer_is_full = 1;
		}
		// this check can be removed - END
				
		if(free_window_ < 0) {
			// should never occur
			printf("[TFRC FC SINK] Rcv ERROR: Free Window is NEGATIVE (%d)! -> abort\n", free_window_);	
			abort();
		}
		
		if(aid > max_aid_) 
			max_aid_ = aid;
	}
	
#ifdef DEBUG_FCSACK					
		print_sackvec();
#endif
	/* END FC SACK TREATMENT */
	
	/* for the time being, we will ignore out of order and duplicate 
	   packets etc. */
	fsize_ = tfrch->fsize;
	int oldmaxseq = maxseq;
	// if this is the highest packet yet, or an unknown packet
	//   between maxseqList and maxseq  
	if ((seqno > maxseq) || 
	  (seqno > maxseqList && lossvec_[seqno%hsz] == UNKNOWN )) {
		if (seqno > maxseqList + 1)
			++ numPktsSoFar_;
		UrgentFlag = tfrch->UrgentFlag;
		round_id = tfrch->round_id ;
		rtt_=tfrch->rtt;
		tzero_=tfrch->tzero;
		psize_=tfrch->psize;
		sendrate = tfrch->rate;
		last_arrival_=now;
		last_timestamp_=tfrch->timestamp;
		rtvec_[seqno%hsz]=now;	
		tsvec_[seqno%hsz]=last_timestamp_;	
		if (hf->ect() == 1 && hf->ce() == 1) {
			// ECN action
			lossvec_[seqno%hsz] = ECN_RCVD;
			++ total_losses_;
			losses_since_last_report++;
			if (new_loss(seqno, tsvec_[seqno%hsz])) {
				ecnEvent = 1;
				lossvec_[seqno%hsz] = ECNLOST;
			} 
			if (algo == WALI) {
                       		++ losses[0];
			}
		} else 
			lossvec_[seqno%hsz] = RCVD;
	}
	if (seqno > maxseq) {
		int i = maxseq + 1;
		while (i < seqno) {
			// Added 3/1/05 in case we have wrapped around
			//   in packet sequence space.
			lossvec_[i%hsz] = UNKNOWN;
			++ i;
			++ total_losses_;
			++ total_dropped_;
		}
	}
	if (seqno > maxseqList && 
	  (ecnEvent || numPktsSoFar_ >= numPkts_ ||
	     tsvec_[seqno%hsz] - tsvec_[maxseqList%hsz] > rtt_)) {
		// numPktsSoFar_ >= numPkts_:
		// Number of pkts since we last entered this procedure
		//   at least equal numPkts_, the number of non-sequential 
		//   packets that must be seen before inferring loss.
		// maxseqList: max seq number checked for dropped packets
		// Decide which losses begin new loss events.
		int i = maxseqList ;
		while(i < seqno) {
			if (lossvec_[i%hsz] == UNKNOWN) {
				rtvec_[i%hsz]=now;	
				tsvec_[i%hsz]=estimate_tstamp(oldmaxseq, seqno, i);	
				if (new_loss(i, tsvec_[i%hsz])) {
					congestionEvent = 1;
					lossvec_[i%hsz] = LOST;
				} else {
					// This lost packet is marked "NOT_RCVD"
					// as it does not begin a loss event.
					lossvec_[i%hsz] = NOT_RCVD; 
				}
				if (algo == WALI) {
			    		++ losses[0];
				}
				losses_since_last_report++;
			}
			i++;
		}
		maxseqList = seqno;
		numPktsSoFar_ = 0;
	} else if (seqno == maxseqList + 1) {
		maxseqList = seqno;
		numPktsSoFar_ = 0;
	} 
	if (seqno > maxseq) {
		maxseq = tfrch->seqno ;
		// if we are in slow start (i.e. (loss_seen_yet ==0)), 
		// and if we saw a loss, report it immediately
		if ((algo == WALI) && (loss_seen_yet ==0) && 
		  (tfrch->seqno - oldmaxseq > 1 || ecnEvent )) {
			UrgentFlag = 1 ; 
			loss_seen_yet = 1;
			if (adjust_history_after_ss) {
				p = adjust_history(tfrch->timestamp);
			}

		}
		if ((rtt_ > SMALLFLOAT) && 
			(now - last_report_sent >= rtt_/NumFeedback_)) {
			UrgentFlag = 1 ;
		}
	}
	
	if (UrgentFlag || ecnEvent || congestionEvent || buffer_is_full) {
		nextpkt(p);
	}

	// data packet delivery to "application", depending if appl. read rate set to unlimit or not
	if(deliver2app || (applreadtimewindow == -1))
		deliver_to_app();

	Packet::free(pkt);
}

double TfrcFcSinkAgent::est_loss () 
{	
	double p = 0 ;
	switch (algo) {
		case WALI:
			p = est_loss_WALI () ;
			break;
		case EWMA:
			p = est_loss_EWMA () ;
			break;
		case RBPH:
			p = est_loss_RBPH () ;
			break;
		case EBPH:
			p = est_loss_EBPH () ;
			break;
		default:
			printf ("invalid algo specified\n");
			abort();
			break ; 
	}
	return p;
}

/*
 * compute estimated throughput in packets per RTT for report.
 */
double TfrcFcSinkAgent::est_thput () 
{
	double time_for_rcv_rate;
	double now = Scheduler::instance().clock();
	double thput = 1 ;
	
	if ((rtt_ > 0) && ((now - last_report_sent) >= rtt_)) {
		// more than an RTT since the last report
		time_for_rcv_rate = (now - last_report_sent);
		if (rcvd_since_last_report > 0) {
			thput = rcvd_since_last_report/time_for_rcv_rate;
		}
	}
	else {
		// count number of packets received in the last RTT
		if (rtt_ > 0){
			double last = rtvec_[maxseq%hsz]; 
			int rcvd = 0;
			int i = maxseq;
			while (i > 0) {
				if (lossvec_[i%hsz] == RCVD) {
					if ((rtvec_[i%hsz] + rtt_) > last) 
						rcvd++; 
					else
						break ;
				}
				i--; 
			}
			if (rcvd > 0)
				thput = rcvd/rtt_; 
		}
	}
	
	return thput ;
}

/*
 * Schedule sending this report, and set timer for the next one.
 */
void TfrcFcSinkAgent::nextpkt(double p) 
{
	sendpkt(p);
	/* schedule next report rtt/NumFeedback_ later */
	/* note from Sally: why is this 1.5 instead of 1.0? */
	/* note from YL: Changed to 1.0 */
	if (rtt_ > 0.0 && NumFeedback_ > 0) 
		nack_timer_.resched(1.0*rtt_/NumFeedback_);
}

/*
 * Create report message, and send it.
 * we create/send SACK, FC and TFRC feedback msg
 */
void TfrcFcSinkAgent::sendpkt(double p)
{
	double now = Scheduler::instance().clock();

	/* don't send a SACK unless we've received new data*/
	/* if we're sending slower than one packet per RTT, don't need*/
    /*
	 * Do we want to send a report even if we have not received
	 * any new data?
     */ 
     
    // if (last_arrival_ >= last_report_sent || sackonly || (rcv_buffer_full_state && (free_window_ > 0))) {
	if (max_aid_ > lastaidSACKsent_ || (rcv_buffer_full_state && (free_window_ > 0))) {
		
		// for Flow Control and SACK, by YL
		int size = (sackvec_length_ * sizeof(int));
		Packet* pkt = allocpkt(size);

		if (pkt == NULL) {
			printf ("error allocating packet\n");
			abort(); 
		}

		// Flow Control and SACK creation by YL
		hdr_tfrc_fc_sack *tfrc_sackh = hdr_tfrc_fc_sack::access(pkt);	// by YL
				
		tfrc_sackh->timestamp = now;	
		if(rcv_buffer_full_state == true) {
			/* the buffer is/was full */
			if(free_window_ > 0) {
#ifdef DEBUG_TRACEFCSACK				
				trace_rcv_buffer_full_state();
#endif				
				rcv_buffer_full_state = false;
				tfrc_sackh->window = free_window_;
			}
			else
				tfrc_sackh->window = 0;
		}
		else
			tfrc_sackh->window = free_window_;
			
		tfrc_sackh->sack_length = sackvec_length_;
		tfrc_sackh->ack = lastaidSACKsent_ + sackvec_[0];
		/* if sackvec_[0] = 0 (UNKNOWN) the vector starts with 0, thus acknowledge the aid just before sackvec_[0] (which was received and sacked)
		 * if sackvec_[0] = 1 (RCVD) then ack is the aid of the first element in the vector (sackvec_[0])
		 */
		
		int* vector = (int*) pkt->accessdata();
		int i;
		for(i = 0; i<sackvec_length_; i++) 
			*vector++ = sackvec_[i];

#ifdef DEBUG_FCSACK			
		/* Print content of SACK msg */
		printf("\n*************** SACK msg *****************\n");
		printf("Time: %f\n win: %d\n ack: %d\n \nsack_length: %d\n| ",
			now, tfrc_sackh->window, tfrc_sackh->ack, tfrc_sackh->sack_length);
		for (int i = 0; i < sackvec_length_; i++)
			printf("%d ", sackvec_[i]);
		printf("|\n*****************************************\n");
#endif

		/* slide/reorder SACK vector:
		 * all elements left side of first 0 in vector are removed from SACK vector 
		 */
		if((sackedsofar_ - lastaidSACKsent_) == sackvec_length_) {
			for (i = 0; i < window_size; i++)
				sackvec_[i] = UNKNOWN;
		}
		else {
			int *sackvec_temp = NULL;
			if((sackvec_temp=(int *)malloc(SIZE_INT*window_size)) != NULL) {
				for (i = 0; i < (max_aid_-sackedsofar_); i++)
					sackvec_temp[i] = sackvec_[i+(sackedsofar_-lastaidSACKsent_)];
				for (i = (max_aid_-sackedsofar_); i < window_size; i++)
					sackvec_temp[i] = UNKNOWN;
				free((char *)sackvec_);
				sackvec_ = sackvec_temp;
			}
			else {
				printf ("error allocating memory for temporary SACK vector\n");
				abort();
			}
		}		
		
		sackvec_length_ -= (sackedsofar_ - lastaidSACKsent_);
		lastaidSACKsent_ = sackedsofar_;
		//-----END FC and SACK------

		// TFRC feedback
		hdr_tfrc_fc_fb *tfrc_fch = hdr_tfrc_fc_fb::access(pkt);
	
		tfrc_fch->seqno=maxseq;
		tfrc_fch->timestamp_echo=last_timestamp_;
		tfrc_fch->timestamp_offset=now-last_arrival_;
//		tfrc_fch->timestamp=now;
		tfrc_fch->NumFeedback_ = NumFeedback_;
		if (p < 0) 
			tfrc_fch->flost = est_loss (); 
		else
			tfrc_fch->flost = p;
		tfrc_fch->rate_since_last_report = est_thput ();
		tfrc_fch->losses = losses_since_last_report;
		if (total_received_ <= 0) 
			tfrc_fch->true_loss = 0.0;
		else 
			tfrc_fch->true_loss = 1.0 * 
			    total_losses_/(total_received_+total_dropped_);
	
		last_report_sent = now; 
		rcvd_since_last_report = 0;
		losses_since_last_report = 0;
				
		send(pkt, 0);
	}
}

int TfrcFcSinkAgent::command(int argc, const char*const* argv) 
{
	if (argc == 3) {
		if (strcmp(argv[1], "weights") == 0) {
			/* 
			 * weights is a string of numbers, seperated by + signs
			 * the firs number is the total number of weights.
			 * the rest of them are the actual weights
			 * this overrides the defaults
			 */
			char *w ;
			w = (char *)calloc(strlen(argv[2])+1, sizeof(char)) ;
			if (w == NULL) {
				printf ("error allocating w\n");
				abort();
			}
			strcpy(w, (char *)argv[2]);
			numsamples = atoi(strtok(w,"+"));
			sample = (int *)malloc((numsamples+1)*sizeof(int));
			losses = (int *)malloc((numsamples+1)*sizeof(int));
            count_losses = (int *)malloc((numsamples+1)*sizeof(int));
			weights = (double *)malloc((numsamples+1)*sizeof(double));
			mult = (double *)malloc((numsamples+1)*sizeof(double));
			fflush(stdout);
			if (sample && weights) {
				int count = 0 ;
				while (count < numsamples) {
					sample[count] = 0;
					losses[count] = 1;
					count_losses[count] = 0;
					mult[count] = 1;
					char *w;
					w = strtok(NULL, "+");
					if (w == NULL)
						break ; 
					else {
						weights[count++] = atof(w);
					}	
				}
				if (count < numsamples) {
					printf ("error in weights string %s\n", argv[2]);
					abort();
				}
				sample[count] = 0;
				losses[count] = 1;
				count_losses[count] = 0;
				weights[count] = 0;
				mult[count] = 1;
				free(w);
				return (TCL_OK);
			}
			else {
				printf ("error allocating memory for smaple and weights:2\n");
				abort();
			}
		}
	}
	return (Agent::command(argc, argv));
}

void TfrcFcNackTimer::expire(Event *) {
	a_->nextpkt(-1);
}

void TfrcFcDeliveryTimer::expire(Event *) {
	b_->deliver_to_app();
}

void TfrcFcSinkAgent::deliver_to_app()
{
	// deliver to appl (deliveredtoapp_ must not be higher sackedsofar_ )
	if(deliveredtoapp_ < sackedsofar_) {
#ifdef DEBUG_TRACEFCSACK		
		trace_buffer_occupancy();
#endif		
		deliveredtoapp_ ++;
		free_window_ ++;
#ifdef DEBUG_FCSACK			
		printf("[TFRC FC SINK] DELIVERY TO APP: %d AID packets deliverd (new free window: %d)\n", deliveredtoapp_, free_window_);
#endif			
		if(free_window_ > window_size) {
			printf("[TFRC FC SINK] ERROR: Free_window is bigger than window_size\n");
			abort();
		}
	}

	/* reschedule the delivery to application rate */
	if(applreadtimewindow != -1)
		delivertimer_.resched(applreadtimewindow);
}

// Tracing functions for FC SACK... START
void TfrcFcSinkAgent::trace_buffer_occupancy()
{
	FILE * pFile;
	double now = Scheduler::instance().clock();
	
	pFile = fopen ("buffer_occupancy_trace.txt","a");
  	if (pFile!=NULL)
  	{
  		// occupancy given in percentage
	    fprintf (pFile, "%f %d\n", now, (int)(100*((double)(window_size-free_window_)/(double)window_size)));
	    fclose (pFile);
  	}
}

void TfrcFcSinkAgent::trace_rcv_buffer_full_state()
{
	FILE * pFile;
	double now = Scheduler::instance().clock();
	
	pFile = fopen ("trace_rcv_buffer_full_state.tr","a");
  	if (pFile!=NULL)
  	{
  		// occupancy given in percentage
	    fprintf (pFile, "%f :: Buffer Full at RCV\n", now);
	    fclose (pFile);
  	}
	
	
}
// Tracing functions for FC SACK... END

void TfrcFcSinkAgent::print_loss(int sample, double ave_interval)
{
	double now = Scheduler::instance().clock();
	double drops = 1/ave_interval;
	// This is ugly to include this twice, but the first one is
	//   for backward compatibility with earlier scripts. 
	printf ("time: %7.5f loss_rate: %7.5f \n", now, drops);
	printf ("time: %7.5f sample 0: %5d loss_rate: %7.5f \n", 
		now, sample, drops);
	//printf ("time: %7.5f send_rate: %7.5f\n", now, sendrate);
	//printf ("time: %7.5f maxseq: %d\n", now, maxseq);
}

void TfrcFcSinkAgent::print_loss_all(int *sample) 
{
	double now = Scheduler::instance().clock();
	printf ("%f: sample 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, sample[0], sample[1], sample[2], sample[3], sample[4]); 
}

void TfrcFcSinkAgent::print_losses_all(int *losses) 
{
	double now = Scheduler::instance().clock();
	printf ("%f: losses 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, losses[0], losses[1], losses[2], losses[3], losses[4]); 
}

void TfrcFcSinkAgent::print_count_losses_all(int *count_losses) 
{
	double now = Scheduler::instance().clock();
	printf ("%f: count? 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, count_losses[0], count_losses[1], count_losses[2], count_losses[3], count_losses[4]); 
}

void TfrcFcSinkAgent::print_sackvec()
{
	double now = Scheduler::instance().clock();
	printf("\n----------- SACK vector after RCV -------------\n");
	printf("Time: %f\n Length: %d\n Win: %d\n lastseqSACKsent: %d\n sackedsofar: %d\n max_aid_: %d\n| ",
		now, sackvec_length_, free_window_, lastaidSACKsent_, sackedsofar_, max_aid_);
	for (int i = 0; i < sackvec_length_; i++)
		printf("%d ", sackvec_[i]);
	printf("|\n----------------------------------------------\n");
}

////////////////////////////////////////
// algo specific code /////////////////
///////////////////////////////////////


////
/// WALI Code
////
double TfrcFcSinkAgent::est_loss_WALI () 
{
	int i;
	double ave_interval1, ave_interval2; 
	int ds ; 
		
	if (!init_WALI_flag) {
		init_WALI () ;
	}
	// sample[i] counts the number of packets since the i-th loss event
	// sample[0] contains the most recent sample.
	for (i = last_sample; i <= maxseq ; i ++) {
		sample[0]++;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST) {
		        //  new loss event
			// double now = Scheduler::instance().clock();
			sample_count ++;
			shift_array (sample, numsamples+1, 0); 
			shift_array (losses, numsamples+1, 1); 
			shift_array (count_losses, numsamples+1, 0); 
			multiply_array(mult, numsamples+1, mult_factor_);
			shift_array (mult, numsamples+1, 1.0); 
			mult_factor_ = 1.0;
		}
	}
	last_sample = maxseq+1 ; 

	if (sample_count>numsamples+1)
		// The array of loss intervals is full.
		ds=numsamples+1;
    	else
		ds=sample_count;

	if (sample_count == 1 && false_sample == 0) 
		// no losses yet
		return 0; 
	/* do we need to discount weights? */
	if (sample_count > 1 && discount && sample[0] > 0) {
                double ave = weighted_average1(1, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses);
                //double ave = weighted_average(1, ds, 1.0, mult, weights, sample);
		int factor = 2;
		double ratio = (factor*ave)/sample[0];
		double min_ratio = 0.5;
		if ( ratio < 1.0) {
			// the most recent loss interval is very large
			mult_factor_ = ratio;
			if (mult_factor_ < min_ratio) 
				mult_factor_ = min_ratio;
		}
	}
	// Calculations including the most recent loss interval.
        ave_interval1 = weighted_average1(0, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses);
        //ave_interval1 = weighted_average(0, ds, mult_factor_, mult, weights, sample);
	// The most recent loss interval does not end in a loss
	// event.  Include the most recent interval in the 
	// calculations only if this increases the estimated loss
	// interval.
        ave_interval2 = weighted_average1(1, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses);
        //ave_interval2 = weighted_average(1, ds, mult_factor_, mult, weights, sample);
	if (ave_interval2 > ave_interval1)
		ave_interval1 = ave_interval2;
	if (ave_interval1 > 0) { 
		if (printLoss_ > 0) {
			print_loss(sample[0], ave_interval1);
			print_loss_all(sample);
			if (ShortIntervals_ > 0) {
				print_losses_all(losses);
				print_count_losses_all(count_losses);
			}
		}
		return 1/ave_interval1; 
	} else return 999;     
}

// Calculate the weighted average.
double TfrcFcSinkAgent::weighted_average(int start, int end, double factor, double *m, double *w, int *sample)
{
	int i; 
	double wsum = 0;
	double answer = 0;
	if (smooth_ == 1 && start == 0) {
		if (end == numsamples+1) {
			// the array is full, but we don't want to uses
			//  the last loss interval in the array
			end = end-1;
		} 
		// effectively shift the weight arrays 
		for (i = start ; i < end; i++) 
			if (i==0)
				wsum += m[i]*w[i+1];
			else 
				wsum += factor*m[i]*w[i+1];
		for (i = start ; i < end; i++)  
			if (i==0)
			 	answer += m[i]*w[i+1]*sample[i]/wsum;
			else 
				answer += factor*m[i]*w[i+1]*sample[i]/wsum;
	        return answer;

	} else {
		for (i = start ; i < end; i++) 
			if (i==0)
				wsum += m[i]*w[i];
			else 
				wsum += factor*m[i]*w[i];
		for (i = start ; i < end; i++)  
			if (i==0)
			 	answer += m[i]*w[i]*sample[i]/wsum;
			else 
				answer += factor*m[i]*w[i]*sample[i]/wsum;
	        return answer;
	}
}

int TfrcFcSinkAgent::get_sample(int oldSample, int numLosses) 
{
	int newSample;
	if (numLosses == 0) {
		newSample = oldSample;
	} else {
		newSample = (int) floor(oldSample / numLosses);
	}
	return newSample;
}

// Calculate the weighted average, factor*m[i]*w[i]*sample[i]/wsum.
// "factor" is "mult_factor_", for weighting the most recent interval
//    when it is very large
// "m[i]" is "mult[]", for old values of "mult_factor_".
//
// When ShortIntervals_ is 1, the length of a loss interval is
//   "sample[i]/losses[i]" for short intervals, not just "sample[i]".
//   This is equivalent to a loss event rate of "losses[i]/sample[i]",
//   instead of "1/sample[i]".
//
// When ShortIntervals_ is 2, it is like ShortIntervals_ of 1,
//   except that the number of losses per loss interval is at
//   most 1460/byte-size-of-small-packets.
//
// When ShortIntervals_ is 2, it is like ShortIntervals_ of 1,
//   except that the number of losses per loss interval is at
//   most 1460/byte-size-of-small-packets, and an interval size is 
//   at most three RTTs.
//
double TfrcFcSinkAgent::weighted_average1(int start, int end, double factor, double *m, double *w, int *sample, int ShortIntervals, int *losses, int *count_losses)
{
        int i;
        int ThisSample;
        double wsum = 0;
        double answer = 0;
        if (smooth_ == 1 && start == 0) {
                if (end == numsamples+1) {
                        // the array is full, but we don't want to uses
                        //  the last loss interval in the array
                        end = end-1;
                }
                // effectively shift the weight arrays
                for (i = start ; i < end; i++)
                        if (i==0)
                                wsum += m[i]*w[i+1];
                        else
                                wsum += factor*m[i]*w[i+1];
                for (i = start ; i < end; i++) {
                        ThisSample = sample[i];
                        if (ShortIntervals == 1 && count_losses[i] == 1) {
			       ThisSample = get_sample(sample[i], losses[i]);
                        }
                        if (ShortIntervals == 2 && count_losses[i] == 1) {
					       int adjusted_losses = int(fsize_/size_);
					       if (losses[i] < adjusted_losses) {
							adjusted_losses = losses[i];
					       }
					       ThisSample = get_sample(sample[i], adjusted_losses);
                        }
                        if (i==0)
                                answer += m[i]*w[i+1]*ThisSample/wsum;
                                //answer += m[i]*w[i+1]*sample[i]/wsum;
                        else
                                answer += factor*m[i]*w[i+1]*ThisSample/wsum;
                                //answer += factor*m[i]*w[i+1]*sample[i]/wsum;
		}
                return answer;

        } else {
                for (i = start ; i < end; i++)
                        if (i==0)
                                wsum += m[i]*w[i];
                        else
                                wsum += factor*m[i]*w[i];
                for (i = start ; i < end; i++) {
                    ThisSample = sample[i];
                    if (ShortIntervals == 1 && count_losses[i] == 1) {
			       		ThisSample = get_sample(sample[i], losses[i]);
                    }
                    if (ShortIntervals == 2 && count_losses[i] == 1) {
			       		ThisSample = get_sample(sample[i], 7);
			       		// Replace 7 by 1460/packet size.
                    }
                    if (i==0)
                    	answer += m[i]*w[i]*ThisSample/wsum;
                        //answer += m[i]*w[i]*sample[i]/wsum;
                    else
                    	answer += factor*m[i]*w[i]*ThisSample/wsum;
                        //answer += factor*m[i]*w[i]*sample[i]/wsum;
				}
                return answer;
        }
}

// Shift array a[] up, starting with a[sz-2] -> a[sz-1].
void TfrcFcSinkAgent::shift_array(int *a, int sz, int defval) 
{
	int i ;
	for (i = sz-2 ; i >= 0 ; i--) {
		a[i+1] = a[i] ;
	}
	a[0] = defval;
}
void TfrcFcSinkAgent::shift_array(double *a, int sz, double defval) {
	int i ;
	for (i = sz-2 ; i >= 0 ; i--) {
		a[i+1] = a[i] ;
	}
	a[0] = defval;
}

// Multiply array by value, starting with array index 1.
// Array index 0 of the unshifted array contains the most recent interval.
void TfrcFcSinkAgent::multiply_array(double *a, int sz, double multiplier) {
	int i ;
	for (i = 1; i <= sz-1; i++) {
		double old = a[i];
		a[i] = old * multiplier ;
	}
}

/*
 * We just received our first loss, and need to adjust our history.
 */
double TfrcFcSinkAgent::adjust_history (double ts)
{
	int i;
	double p;
	for (i = maxseq; i >= 0 ; i --) {
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST ) {
			lossvec_[i%hsz] = NOT_RCVD; 
		}
	}
	lastloss = ts; 
	lastloss_round_id = round_id ;
	p = b_to_p2(est_thput()*psize_, rtt_, tzero_, fsize_, 1);	// YLYLYL
	false_sample = (int)(1.0/p);
	sample[1] = false_sample;
	sample[0] = 0;
	losses[1] = 0;
	losses[0] = 1;
	count_losses[1] = 0;
	count_losses[0] = 0;
	sample_count++; 
	if (printLoss_) {
		print_loss_all (sample);
		if (ShortIntervals_ == 1) {
			print_losses_all(losses);
			print_count_losses_all(count_losses);
		}
	}
	false_sample = -1 ; 
	return p;
}


/*
 * Initialize data structures for weights.
 */
void TfrcFcSinkAgent::init_WALI () {
	int i;
	if (numsamples < 0)
		numsamples = DEFAULT_NUMSAMPLES ;	
	if (smooth_ == 1) {
		numsamples = numsamples + 1;
	}
	sample = (int *)malloc((numsamples+1)*sizeof(int));
    losses = (int *)malloc((numsamples+1)*sizeof(int));
    count_losses = (int *)malloc((numsamples+1)*sizeof(int));
	weights = (double *)malloc((numsamples+1)*sizeof(double));
	mult = (double *)malloc((numsamples+1)*sizeof(double));
	for (i = 0 ; i < numsamples+1 ; i ++) {
		sample[i] = 0 ; 
	}
	if (smooth_ == 1) {
		int mid = int(numsamples/2);
		for (i = 0; i < mid; i ++) {
			weights[i] = 1.0;
		}
		for (i = mid; i <= numsamples; i ++){
			weights[i] = 1.0 - (i-mid)/(mid + 1.0);
		}
	} else {
		int mid = int(numsamples/2);
		for (i = 0; i < mid; i ++) {
			weights[i] = 1.0;
		}
		for (i = mid; i <= numsamples; i ++){
			weights[i] = 1.0 - (i+1-mid)/(mid + 1.0);
		}
	}
	for (i = 0; i < numsamples+1; i ++) {
		mult[i] = 1.0 ; 
	}
	init_WALI_flag = 1;  /* initialization done */
}

///////////////////////////
// EWMA //////////////////
//////////////////////////

double TfrcFcSinkAgent::est_loss_EWMA () {
	double p1, p2 ;
	for (int i = last_sample; i <= maxseq ; i ++) {
		loss_int++; 
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST ) {
			if (avg_loss_int < 0) {
				avg_loss_int = loss_int ; 
			} else {
				avg_loss_int = history*avg_loss_int + (1-history)*loss_int ;
			}
			loss_int = 0 ;
		}
	}
	last_sample = maxseq+1 ; 

	if (avg_loss_int < 0) { 
		p1 = 0;
	} else {
		p1 = 1.0/avg_loss_int ; 
	}
	if (loss_int == 0 
	    || avg_loss_int < 0){ //XXX this last check was added by a
				  //person who knows nothing of this
				  //code just to stop FP div by zero.
				  //Values were history=.75,
				  //avg_loss_int=-1, loss_int=3.  If
				  //you know what should be here,
				  //please cleanup and remove this
				  //comment.

		p2 = p1 ; 
	} else {
		p2 = 1.0/(history*avg_loss_int + (1-history)*loss_int) ;
	}
	if (p2 < p1) {
		p1 = p2 ; 
	}
	if (printLoss_ > 0) {
		if (p1 > 0) 
			print_loss(loss_int, 1.0/p1);
		else
			print_loss(loss_int, 0.00001);
		print_loss_all(sample);
	}
	return p1 ;
}

///////////////////////////
// RBPH //////////////////
//////////////////////////
double TfrcFcSinkAgent::est_loss_RBPH () {
	double numpkts = hsz ;
	double p ; 

	// how many pkts we should go back?
	if (sendrate > 0 && rtt_ > 0) {	
		double x = b_to_p2(sendrate, rtt_, tzero_, psize_, 1);	// YLYLYL
		if (x > 0) 
			numpkts = minlc/x ; 
		else
			numpkts = hsz ;
	}

	// that number must be below maxseq and hsz 
	if (numpkts > maxseq)
		numpkts = maxseq ;
	if (numpkts > hsz)
		numpkts = hsz ;

	int lc = 0;
	int pc = 0;
	int i = maxseq ;

	// first see if how many lc's we find in numpkts 
	while (pc < numpkts) {
		pc ++ ;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST )
			lc ++ ; 
		i -- ;
	}

	// if not enough lsos events, keep going back ...
	if (lc < minlc) {

		// but only as far as the history allows ...
		numpkts = maxseq ;
		if (numpkts > hsz)
			numpkts = hsz ;

		while ((lc < minlc) && (pc < numpkts)) {
			pc ++ ;
			if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST )
				lc ++ ;
			i -- ;
		
		}
	}

	if (pc == 0) 
		p = 0; 
	else
		p = (double)lc/(double)pc ; 
	if (printLoss_ > 0) {
		if (p > 0) 
			print_loss(0, 1.0/p);
		else
			print_loss(0, 0.00001);
		print_loss_all(sample);
	}
	return p ;
}

///////////////////////////
// EBPH //////////////////
//////////////////////////
double TfrcFcSinkAgent::est_loss_EBPH () {
	double numpkts = hsz ;
	double p ; 

	int lc = 0;
	int pc = 0;
	int i = maxseq ;

	numpkts = maxseq ;
	if (numpkts > hsz)
		numpkts = hsz ;

	while ((lc < minlc) && (pc < numpkts)) {
		pc ++ ;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST)
			lc ++ ;
		i -- ;
	}

	if (pc == 0) 
		p = 0; 
	else
		p = (double)lc/(double)pc ; 
	if (printLoss_ > 0) {
		if (p > 0) 
			print_loss(0, 1.0/p);
		else
			print_loss(0, 0.00001);
		print_loss_all(sample);
	}
	return p ;
}

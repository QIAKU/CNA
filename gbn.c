#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"
#include <string.h>


/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */


static int acked[SEQSPACE];    /* Marks whether each packet has received ACK*/
static float timers[SEQSPACE];  /* The timer start time for each packet*/
static bool timer_active[SEQSPACE]; /* Mark whether the timer of each package is activated*/
static float current_time=0.0; /* Simulation time*/



/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{ 
  /*If the current window is full, no new packets can be sent.*/
  if(windowcount>= WINDOWSIZE){
    if(TRACE >0){
      printf("----A: Window full. Can not send message.\n");
      return;
    };
  }

  /*The current serial number to be used*/
  int seq= A_nextseqnum;

  /*Construct a data packet*/
  struct pkt packet;  /*Set serial number*/ 
  packet.seqnum = seq; /*Set the sequence number of the data packet, which is used by the receiver to determine whether it is received in order*/
  packet.acknum =0;  /* ACK field is set to 0*/
  memcpy(packet.payload, message.data, sizeof(message.data));  /* Copy upper layer data*/
  packet.checksum = ComputeChecksum(packet); /* Calculate checksum*/

  buffer[seq] =packet;  /* Save this packet to the send buffer*/
  acked[seq] =0;   /* The packet has not received ACK yet and is marked as unconfirmed.*/
  timer_active[seq]=true;  /* Start the timer marker for this packet */
  timers[seq]= current_time;  /* Record the simulator time when the package starts the timer*/

  tolayer3(A, packet);   /* Sending the packet to the network layer*/
  if (TRACE > 0)
        printf("----A: Sent packet %d\n", seq);
  
  A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  /* Update the next available serial number*/
  windowcount++;   /* The number of packets to be confirmed in the current window +1*/
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  /* If the received ACK is corrupted, it is ignored */
  if(IsCorrupted(packet)){
    if(TRACE>0){
      printf("----A: Corrupted ACK %d received, ignored.\n", packet.acknum);
      return;
    }
  }

  /* Get the sequence number corresponding to the ACK */
  int acknum =packet.acknum;

  /* If this ACK is received for the first time */
  if(!acked[acknum]){
    acked[acknum]=1;  /* Mark the serial number as confirmed */
    timer_active[acknum]=false; /* Stop the timer tag corresponding to the packet */

    if(TRACE>0){
      printf("----A: ACK %d marked as received\n", acknum); /*If TRACE mode is enabled, print the sequence number of the successfully received ACK for debugging*/
    }

    /*Slide the window to the right - starting from the left, continuously release confirmed packets*/
    while (acked[windowfirst])
    {
      acked[windowfirst]=0; /*Reset confirmation status*/
      timer_active[windowfirst]=false; /*reset timing mark*/

      if(TRACE > 0){
        printf("----A: Sliding window, freeing packet %d\n", windowfirst); 
      }

      windowfirst =(windowfirst+1)%SEQSPACE; /*Slide window right*/
      windowcount--; /*A packet has been acknowledged, and the window count is reduced by 1*/
    }
    
    /* If there are still packets that have not been confirmed, restart the unique timer, otherwise stop */
    stoptimer(A);
    /*If there are still unconfirmed packets in the window, it means there are still tasks waiting for ACK*/
    if(windowcount>0){
      /*Restart the timer and continue monitoring the next unconfirmed packet*/
      starttimer(A,RTT);
      /*Debug information: Timer restart after ACK*/
      if(TRACE > 0){
        printf("----A: Timer restarted after ACK\n");
      }else{
        /*If all packets in the window have been confirmed, the timer is no longer needed*/
        if(TRACE>0){
          printf("----A: All packets acknowledged, timer stopped\n");
        }
      }
    }
    else{
      /*If a duplicate ACK is received (the ACK has already been processed), it is ignored*/
      if(TRACE>0){
        printf("----A: Duplicate ACK %d received, ignored\n", acknum);
      }
    }
  }





}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  int oldest_index=-1; /*Records the index of the earliest packet that started the timer but did not receive an ACK*/
  float oldest_time=1e9;/*Used to find the earliest time to start the timer, initially set to a large value*/

  /*Traverse all packets in the sequence number space and find the earliest packet that started the timer but has not been confirmed*/
  for (i=0; i<SEQSPACE; i++) {
    if (timer_active[i] && timers[i] < oldest_time) {
        oldest_time = timers[i]; /*Update the earliest timer time*/
        oldest_index = i; /*Record the index of the package*/
    }
  }

  /*If a timeout packet is found*/
  if (oldest_index != -1) {
    /*Retransmit the timed-out packet*/
    tolayer3(A, buffer[oldest_index]);

    /*If TRACE debugging mode is enabled, retransmission information is output*/
    if (TRACE > 0)
        printf("----A: Timeout! Resending packet %d\n", oldest_index);

    /*Update the timer start time of this package to the current simulation time*/
    timers[oldest_index] = current_time;

    /*Restart the only global timer*/
    starttimer(A, RTT);
  }


}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;

  /* if not corrupted and received packet is in order */
  if  ( (!IsCorrupted(packet))  && (packet.seqnum == expectedseqnum) ) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    packets_received++;

    /* deliver to receiving application */
    tolayer5(B, packet.payload);

    /* send an ACK for the received packet */
    sendpkt.acknum = expectedseqnum;

    /* update state variables */
    expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
  }
  else {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    if (expectedseqnum == 0)
      sendpkt.acknum = SEQSPACE - 1;
    else
      sendpkt.acknum = expectedseqnum - 1;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  /* we don't have any data to send.  fill payload with 0's */
  for ( i=0; i<20 ; i++ )
    sendpkt.payload[i] = '0';

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt);

  /* send out packet */
  tolayer3 (B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}


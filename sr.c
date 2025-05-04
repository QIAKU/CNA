#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"
#include <string.h>


/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
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
#define SEQSPACE 12      /* The SR protocol requires that the sequence number space be at least twice the window size */
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

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{ 
  int seq;
  struct pkt sendpkt;  /*Set serial number*/ 
  /*If the current window is full, no new packets can be sent.*/
  if(windowcount>= WINDOWSIZE){
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++; /*Add statistics when the window is full*/
    return;
  }
  

  if (TRACE > 1)
  printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");
  /*The current serial number to be used*/
  seq= A_nextseqnum;

  /*Construct a data packet*/
  sendpkt.seqnum = seq; /*Set the sequence number of the data packet, which is used by the receiver to determine whether it is received in order*/
  sendpkt.acknum =0;  /* ACK field is set to 0*/
  memcpy(sendpkt.payload, message.data, sizeof(message.data));  /* Copy upper layer data*/
  sendpkt.checksum = ComputeChecksum(sendpkt); /* Calculate checksum*/

  buffer[seq] =sendpkt;  /* Save this packet to the send buffer*/
  acked[seq] =0;   /* The packet has not received ACK yet and is marked as unconfirmed.*/

  if (TRACE > 0)
  printf("Sending packet %d to layer 3\n", sendpkt.seqnum);

  tolayer3(A, sendpkt);   /* Sending the packet to the network layer*/
  
  if (windowcount == 0) {
    starttimer(A, RTT);
  }

  A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  /* Update the next available serial number*/
  windowcount++;   /* The number of packets to be confirmed in the current window +1*/
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int acknum=packet.acknum;

  /* If the received ACK is corrupted, it is ignored */
  if(IsCorrupted(packet)){
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
    return;
  }

  if (TRACE > 0)
  printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
  total_ACKs_received++;  /*Count the number of all non-damaged ACKs received by end A*/

  /* If this ACK is received for the first time */
  if(!acked[acknum]){
    new_ACKs++;  /*When a new and non-duplicate ACK is received, count the number of new ACKs*/
    if (TRACE > 0)
    printf("----A: ACK %d is not a duplicate\n",packet.acknum);
    acked[acknum]=1;  /* Mark the serial number as confirmed */

    /*Move the window's starting sequence number windowfirst to the right, skipping the confirmed packets*/
    while(acked[windowfirst])
    {
      windowfirst =(windowfirst+1)%SEQSPACE; /*Slide window right*/
      windowcount--; /*A packet has been acknowledged, and the window count is reduced by 1*/
    }
    
    /* If there are still packets that have not been confirmed, restart the unique timer, otherwise stop */
    stoptimer(A);

    /*If there are still unconfirmed packets in the window, it means there are still tasks waiting for ACK*/
    if (windowcount > 0) {
      starttimer(A, RTT);
    } 
  }
  else
  if (TRACE > 0)
  printf ("----A: duplicate ACK received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
    /*Only the first unacknowledged packet in the retransmission window*/ 
    int seq;
    int i;

    if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

    for (i = 0; i < windowcount; i++) {

      seq = (windowfirst + i) % SEQSPACE;
      if (!acked[seq]) {
        if (TRACE > 0)
        printf ("---A: resending packet %d\n", (buffer[(windowfirst+i) % SEQSPACE]).seqnum);
        tolayer3(A, buffer[seq]);
        packets_resent++; /*When A retransmits a packet due to timeout, count the number of retransmissions*/
      }
    }
    stoptimer(A);
    if(windowcount > 0){
      starttimer(A, RTT);
    }
    
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  int i;
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;
  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = 0;
    memset(&buffer[i], 0, sizeof(struct pkt));
    }
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt B_buffer[SEQSPACE]; /*Cache received packets*/
static bool B_received[SEQSPACE]; /*Mark whether the serial number has been received*/ 

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt ackpkt; /*Define a packet for sending ACK*/
  int i;
  int seq;
  int seqfirst;
  int seqlast;
  bool inWindow;

  seq = packet.seqnum;/*The sequence number of the currently received data packet*/

  /*If the received packet is not corrupted*/
  if (!IsCorrupted(packet)) {
    
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    
    /*Determine whether the data packet with this sequence number is within the receiving window range*/
    seqfirst=expectedseqnum;
    seqlast=(expectedseqnum+WINDOWSIZE-1)%SEQSPACE;
    
    if(seqfirst<=seqlast){
      /*Normal situation where sequence number space wrapping does not occur*/
      inWindow=(seq>=seqfirst && seq<=seqlast);
    }else{
      /*expectedseqnum + window size exceeds the maximum sequence number*/
      inWindow=(seq>=seqfirst || seq<=seqlast);
    }

    if(inWindow){
      /*If the received packet is within the current receive window*/
      if(!B_received[seq]){
        /*If the data packet with this sequence number is received for the first time, the packet is cached*/
        B_buffer[seq]=packet;/*Store in buffer*/
        B_received[seq]=true;/*Mark the serial number received*/
      }
      packets_received++; /*Count the number of correct data packets successfully received by end B*/

      /*The response sequence number is the sequence number of the data packet.*/
      ackpkt.acknum=seq;
    }else{
      if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
      /*When a data packet is damaged, the ACK of the last packet received in sequence is still resent*/
      ackpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    } 
  }else{
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
      /*When a data packet is damaged, the ACK of the last packet received in sequence is still resent*/
    ackpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    }

    ackpkt.seqnum = 0; /*The ACK packet itself does not carry valid data, so seqnum can be set to 0*/
    for (i = 0; i < 20; i++) /*Fill the ACK payload with the default value*/
        ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);/*Calculate checksum*/
    tolayer3(B, ackpkt); /*Send ACK back to sender A*/

    /*If the expected packets are received, they are delivered one by one*/
    while (B_received[expectedseqnum]) {
        tolayer5(B, B_buffer[expectedseqnum].payload); /*Delivered to the application layer*/
        B_received[expectedseqnum] = false; /*clear mark*/
        expectedseqnum = (expectedseqnum + 1) % SEQSPACE; /*Update expected sequence number*/
    }
}



/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void){
  int i;
  expectedseqnum = 0; /*Initialize the expected received sequence number to 0*/
  B_nextseqnum = 1;  /*Sequence number generation for ACK packets*/
  for (i = 0; i < SEQSPACE; i++) {
    B_received[i] = false; 
    memset(&B_buffer[i], 0, sizeof(struct pkt));
}
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


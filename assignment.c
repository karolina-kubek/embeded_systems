#include "contiki.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/sht11-sensor.h"
#include <math.h>
#include <stdio.h>
#include "cc2420.h"

static int val;
static float s = 0;
static float temp_avg = 0; 
static float humid_avg = 0;
static int maxneighbor = 0;

static uint16_t max_rssi = 0;
static int i = 0;
/* This is the structure of broadcast messages. */
struct broadcast_message {
  uint8_t seqno;
};

/* This is the structure of unicast ping messages. */
struct unicast_message {

 uint8_t temp_dec;
 uint8_t temp_frac;
 uint8_t humid_dec;
 uint8_t humid_frac;
 
};


/* This structure holds information about neighbors. */
struct neighbor {

  struct neighbor *next;
  rimeaddr_t addr;
  uint16_t last_rssi, last_lqi;
  uint8_t last_seqno;
  uint32_t avg_seqno_gap;

};

/* This #define defines the maximum amount of neighbors we can remember. */
#define MAX_NEIGHBORS 20

/* This MEMB() definition defines a memory pool from which we allocate
   neighbor entries. */
MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);

/* The neighbors_list is a Contiki list that holds the neighbors we
   have seen thus far. */
LIST(neighbors_list);

/* These hold the broadcast and unicast structures, respectively. */
static struct broadcast_conn broadcast;
static struct unicast_conn unicast;

/* These two defines are used for computing the moving average for the
   broadcast sequence number gaps. */
#define SEQNO_EWMA_UNITY 0x100
#define SEQNO_EWMA_ALPHA 0x040

/*---------------------------------------------------------------------------*/
/* We first declare our two processes. */
PROCESS(broadcast_process, "Broadcast process");
PROCESS(unicast_process, "Unicast process");

/* The AUTOSTART_PROCESSES() definition specifices what processes to
   start when this module is loaded. We put both our processes
   there. */
AUTOSTART_PROCESSES(&broadcast_process, &unicast_process);
/*---------------------------------------------------------------------------*/
/* This function is called whenever a broadcast message is received. */
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct neighbor *n;
  struct broadcast_message *m;
  uint8_t seqno_gap;

  /* The packetbuf_dataptr() returns a pointer to the first data byte
     in the received packet. */
  m = packetbuf_dataptr();

  /* Check if we already know this neighbor. */
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {

    /* We break out of the loop if the address of the neighbor matches
       the address of the neighbor from which we received this
       broadcast message. */
    if(rimeaddr_cmp(&n->addr, from)) {
      break;
    }
  }

  /* If n is NULL, this neighbor was not found in our list, and we
     allocate a new struct neighbor from the neighbors_memb memory
     pool. */
  if(n == NULL) {
    n = memb_alloc(&neighbors_memb);

    /* If we could not allocate a new neighbor entry, we give up. We
       could have reused an old neighbor entry, but we do not do this
       for now. */
    if(n == NULL) {
      return;
    }

    /* Initialize the fields. */
    rimeaddr_copy(&n->addr, from);
    n->last_seqno = m->seqno - 1;
    n->avg_seqno_gap = SEQNO_EWMA_UNITY;

    /* Place the neighbor on the neighbor list. */
    list_add(neighbors_list, n);
  }

  /* We can now fill in the fields in our neighbor entry. */
  n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);

  /* Compute the average sequence number gap we have seen from this neighbor. */
  seqno_gap = m->seqno - n->last_seqno;
  n->avg_seqno_gap = (((uint32_t)seqno_gap * SEQNO_EWMA_UNITY) *
                      SEQNO_EWMA_ALPHA) / SEQNO_EWMA_UNITY +
                      ((uint32_t)n->avg_seqno_gap * (SEQNO_EWMA_UNITY -
                                                     SEQNO_EWMA_ALPHA)) /
    SEQNO_EWMA_UNITY;

  /* Remember last seqno we heard. */
  n->last_seqno = m->seqno;

  /* Print out a message. */
  printf("Broadcast message received from %d.%d with seqno %d, RSSI %u, LQI %u\n\n",
         from->u8[0], from->u8[1],
         m->seqno,
         n->last_rssi,
         n->last_lqi);
}
/* This is where we define what function to be called when a broadcast
   is received. We pass a pointer to this structure in the
   broadcast_open() call below. */
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
/*---------------------------------------------------------------------------*/
/* This function is called for every incoming unicast packet. */
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
  struct unicast_message *msg;
  /*When a mote receives a packet, the node turns on the green LED.*/
	leds_on(LEDS_GREEN);
	
  // Grab the pointer to the incoming data. 
  msg = packetbuf_dataptr();

  printf("Unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
	printf("Received Temperature=%u.%02u C \n", msg->temp_dec, msg->temp_frac);
  printf("Received Humidity=%u.%02u %% \n\n", msg->humid_dec, msg->temp_frac);
  //leds_off(3);
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
  static struct etimer et;
  static uint8_t seqno;
  struct broadcast_message msg;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  /*Each mote sends periodically a broadcast. However, it does not send any broadcast 			
	  message until the user click the button.*/
	printf("Click the button to start the broadcast\n");
	SENSORS_ACTIVATE(button_sensor);
	PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));
	printf("Broadcast starting...\n");

  broadcast_open(&broadcast, 129, &broadcast_call);

  while(1) {

    /* Send a broadcast every 4 - 8 seconds */
    etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    msg.seqno = seqno;
    packetbuf_copyfrom(&msg, sizeof(struct broadcast_message));
    broadcast_send(&broadcast);
    seqno++;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&unicast);)
    
  PROCESS_BEGIN();

  unicast_open(&unicast, 146, &unicast_callbacks);
	
  while(1) {
    static struct etimer et;
    static struct unicast_message msg;
    static struct neighbor *n;
    
			
		/*Each mote samples temperature and humidity and computes its average over 5 samples.*/
		for(i = 0; i < 5; i = i + 1) {
		
			etimer_set(&et, CLOCK_SECOND * 4);
					
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		
			SENSORS_ACTIVATE(sht11_sensor);
			
			val = sht11_sensor.value(SHT11_SENSOR_TEMP);
		
			if(val != -1)
			{
				s= ((0.01*val) - 39.60);
				temp_avg = temp_avg + s;
			}
			val=sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
		
			if(val != -1)
			{
				s= (((0.0405*val) - 4) + ((-2.8 * 0.000001)*(pow(val,2))));
				humid_avg = humid_avg + s;
			}
		
			etimer_reset(&et);
			SENSORS_DEACTIVATE(sht11_sensor);
		}
	
		temp_avg = temp_avg / 5;
		msg.temp_dec = temp_avg;
	  msg.temp_frac = (unsigned int)temp_avg - msg.temp_dec;
		//printf("\nTemperature=%d.%02u C \n", msg.temp_dec, (unsigned int)(msg.temp_frac * 100));
		humid_avg = humid_avg / 5;
		msg.humid_dec = humid_avg;
	  msg.humid_frac = (unsigned int)humid_avg - msg.humid_dec;
		//printf("Humidity=%d.%02u %% \n", msg.humid_dec, (unsigned int)(msg.humid_frac * 100));

			
		if(list_length(neighbors_list) > 0) {
			/*Each mote sends its sample to its neighbour with the highest RSSI value*/
		
			i = 0;
			for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {
				
				if (n->last_rssi > max_rssi) {
					max_rssi = n->last_rssi;
					maxneighbor = i;
				}
				i = i + 1;
			}
			printf("Max RSSI value: %u \n", max_rssi);
			n = list_head(neighbors_list);
      for(i = 0; i < maxneighbor; i++) {
        n = list_item_next(n);
      }
      
      printf("Sending unicast with samples to %d.%d\n\n", n->addr.u8[0], n->addr.u8[1]);
      packetbuf_copyfrom(&msg, sizeof(msg));
      unicast_send(&unicast, &n->addr);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

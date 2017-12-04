#include "contiki.h"
#include "net/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/sht11-sensor.h"
#include <math.h>
#include <stdio.h>
#include "lib/list.h"

/*struct RSSI_LQI_list_struct {
	struct *next;
	int addr1;
	int addr2;
	int rssi;
	int lqi;
};
*/

//LIST(example_list);

//static struct RSSI_LQI_list_struct element1, element2;

//static struct etimer et;
static int val;
static double s = 0;
static int n = 0;
static double temp_avg = 0; 
static double humid_avg = 0;
static int rssi = 0;
static int lqi = 0;

//Each mote sends periodically a broadcast. However, it does not send any broadcast message until the user click the button.
/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "Broadcast example");
AUTOSTART_PROCESSES(&example_broadcast_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{	
	rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
	printf("broadcast message received from %d.%d: '%s' RSSI: '%d' LQI: '%d'\n",
	from->u8[0], from->u8[1], (char *)packetbuf_dataptr()), rssi, lqi;

}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_broadcast_process, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	printf("Click the button to start\n");
	SENSORS_ACTIVATE(button_sensor);
	PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));
	printf("Broadcast starting...\n");
	broadcast_open(&broadcast, 129, &broadcast_call);
	while(1) {
		/* Delay 2-4 seconds */
		etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		
		
		// compute_averages();
		while( n < 5 )
		{
			etimer_set(&et, CLOCK_SECOND * 2);
			SENSORS_ACTIVATE(sht11_sensor);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
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
	
			n = n + 1;
		}
		n = 0;
		temp_avg = temp_avg / 5;
		printf("\nTemperature=%lf C \n", temp_avg);
		humid_avg = humid_avg / 5;
		printf("Humidity=%lf %% \n", humid_avg);
		
		packetbuf_copyfrom("Bean", 5);
		broadcast_send(&broadcast);
		printf("broadcast message sent\n");
	}
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/

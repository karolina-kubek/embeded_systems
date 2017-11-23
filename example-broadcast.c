#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"
                  
#include "dev/sht11-sensor.h"

#include <math.h>
#include <stdio.h>

//Each mote sends periodically a broadcast. However, it does not send any broadcast message until the user click the button.
/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "Broadcast example");
AUTOSTART_PROCESSES(&example_broadcast_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
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

    packetbuf_copyfrom("Karolina", 9);
    broadcast_send(&broadcast);
    printf("broadcast message sent\n");
  }

  PROCESS_END();
}

//Each mote periodically samples temperature and humidity and computes its average over 5 samples.
PROCESS(sensor_acq_process,"Sensor Acquisition");
AUTOSTART_PROCESSES(&sensor_acq_process);
PROCESS_THREAD(sensor_acq_process,ev,data)
{ 
      static struct etimer et;
      static int val;
      static float s = 0;
      static int dec;
      static float frac;

      PROCESS_BEGIN();

      printf("Starting Sensor Example.\n");
      
      while(1)
      {
		etimer_set(&et, CLOCK_SECOND * 5);
		SENSORS_ACTIVATE(sht11_sensor);
        
	   PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));


           val = sht11_sensor.value(SHT11_SENSOR_TEMP);
      	   if(val != -1) 
      	   {
			s= ((0.01*val) - 39.60);
      	  	dec = s;
      	  	frac = s - dec;
      	  	printf("\nTemperature=%d.%02u C (%d)\n", dec, (unsigned int)(frac * 100),val);               
           }

	   val=sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
	   if(val != -1) 
      	   {
			s= (((0.0405*val) - 4) + ((-2.8 * 0.000001)*(pow(val,2))));  
      	  	dec = s;
      	  	frac = s - dec;
      	  	printf("Humidity=%d.%02u %% (%d)\n", dec, (unsigned int)(frac * 100),val);               
           }

           
	
	   etimer_reset(&et);
    	   SENSORS_DEACTIVATE(sht11_sensor);

      } //end of while
    
      PROCESS_END();
/*---------------------------------------------------------------------------*/

/*
 * ASSUMPTIONS:
 * The initial state is: alarm OFF, gate locked, door closed, garden lights turned off, windows closed
 * Possible states are:		( D = DEAFULT )
 * 	alarm:				OFF(D) - ON
 * 	door:				CLOSED(D) - AUTO_OPENING
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "stdio.h" /* For printf() */
#include "stdlib.h"
#include "dev/button-sensor.h"
#include "net/rime/rime.h"
#include "string.h"
#include "dev/leds.h"
#include "dev/sht11/sht11-sensor.h"
#include "stdbool.h"


#define MAX_RETRANSMISSIONS 			5
#define MSG_LENGTH						8

#define SAMPLING_TEMPERATURE_PERIOD		CLOCK_SECOND*10
#define BLINKIN_PERIOD					CLOCK_SECOND*2
#define OPENING_PERIOD					8

//Address
#define CU_NODE_ADDRESS_0			3
#define CU_NODE_ADDRESS_1			0

/*-----------------  Queue Structure  ------------------------------------------------------------------------------------------------*/
#define QUEUE_ELEMENTS 			5

//the actual circular queue
int queue[ QUEUE_ELEMENTS ];

//Used to store the position that will be used for inserting the next element
uint8_t queue_insert_index;

//This method initializes the queue. The first element will be put in position 0.
void queue_init(){
	queue_insert_index = 0;
	uint8_t i;
	for(i = 0; i < QUEUE_ELEMENTS; i++){
		queue[i] = 0;
	}
}

/*
 * Insert the given element in the 'queue_insert_index' position and set the index of the next value to be inserted.
 * We use a circular queue, thus after using the last position of the array we will override the first position.
 */
void queue_insert(int new){
	queue[queue_insert_index] = new;
	queue_insert_index = (queue_insert_index + 1) % QUEUE_ELEMENTS;
}

//This method simply computes the mean value of the temperature values stored in the queue.
int queue_mean_get(){
	int sum = 0;
	// Queue of max 256 elements, here we need only 5
	uint8_t i;
	for(i = 0; i<QUEUE_ELEMENTS; i++){
		sum += queue[i];
	}
	return (sum/QUEUE_ELEMENTS);
}
/*---------------------------------------------------------------------------*/

static process_event_t message_from_central_unit;
static process_event_t alarm_blink;
static process_event_t opening_blink;
static process_event_t end_blinking;
static process_event_t button_event;

//Enumerator use to make easy the identification of position in home_status array
static enum home_t { ALARM, DOOR, GARDEN_LIGHTS } node;

//Array used to store the state of the system. They are used to perform operations in a consistent manner and for printing only the correct subset of the commands as available commands.
static enum status_t { ON, OFF, AUTO_OPENING, LOCKED, UNLOCKED, MOVING, OPEN, CLOSED, UNAVAILABLE } home_status[ 3 ];

//To maintain the previous status before the insertion of MODIFYING STATUS COMMANDS, g.e. ALARM, AUTO_OPENING
static enum status_t previous_status[ 3 ];

static uint8_t remained_seconds = 0;

//Set GREEN LED = ON, RED LED = OFF
void turn_on_garden_lights() {
	leds_on(  LEDS_GREEN );
	leds_off( LEDS_RED );
	//leds_off( LEDS_BLUE );
}

//Set GREEN LED = OFF, RED LED = ON
void turn_off_garden_lights() {
	leds_off( LEDS_GREEN );
	leds_on(  LEDS_RED );
	//leds_off( LEDS_BLUE );
}

//Store the current state before the insertion of current command
void store_status( bool alarm ) {
	if( alarm == false ) {
		previous_status[ DOOR ] = 			home_status [ DOOR ];
		previous_status[ GARDEN_LIGHTS ] =  home_status [ GARDEN_LIGHTS ];
	}//If the alarm is activated during opening process, the state has not be store! because previuos = auto_opening... overwriting the previous one
}

//Restore the previous state after the insertion of last command g.e. ALARM or AUTO_OPENING
void restore_status(  ) {

	home_status[ DOOR ] = previous_status [ DOOR ];
	home_status[ GARDEN_LIGHTS ] = previous_status [ GARDEN_LIGHTS ];
	// leds has to return in their previous state

	if( home_status[ DOOR ] == AUTO_OPENING )
		leds_on(  LEDS_BLUE );
	else
		leds_off( LEDS_BLUE );

	if( home_status[ GARDEN_LIGHTS ] == ON )
		turn_on_garden_lights();
	else
		turn_off_garden_lights();
}

//The initial state is: alarm deactivated, door closed, garden lights turned off.
void home_status_init() {
	node = ALARM;//NOTHING IMPORTANT, JUST TO MAKE HAPPY THE COMPILER BECAUSE OTHERWISE EMITS UNUSAGE WARNING
	home_status [ ALARM ] 			= OFF;
	home_status [ DOOR ] 			= CLOSED;
	home_status [ GARDEN_LIGHTS ] 	= OFF;

	turn_off_garden_lights();// At the beginning, garden lights are turned off.
	leds_off( LEDS_BLUE );

	previous_status[ ALARM ] =  home_status [ ALARM ];
	previous_status[ DOOR ] = home_status [ DOOR ];
	previous_status[ GARDEN_LIGHTS ] = home_status [ GARDEN_LIGHTS ];
}

//Utility function that print a certain status
void print_state( enum status_t status ) {
	switch (status ) {
	case ON:
		printf("ON");
		break;
	case OFF:
		printf("OFF");
		break;
	case AUTO_OPENING:
		printf("AUTO_OPENING");
		break;
	case LOCKED:
		printf("LOCKED");
		break;
	case UNLOCKED:
		printf("UNLOCKED");
		break;
	case MOVING:
		printf("MOVING");
		break;
	case OPEN:
		printf("OPEN");
		break;
	case CLOSED:
		printf("CLOSED");
		break;
	case UNAVAILABLE:
		printf("UNAVAILABLE");
		break;
	}
	printf("\n");

}

//Internal commands for convenience and internal program usage
static enum internal_command_t {
	ALARM_ON, ALARM_OFF,
	GATE_LOCK, GATE_UNLOCK,
	START_AUTO_OPENING, END_AUTO_OPENING,//END AUTO_OPENING is ONLY received from CU. NEVER SENT!
	INTERNAL_TEMP,
	EXTERNAL_LIGHT,
	GARAGE_OPEN, GARAGE_CLOSED, GARAGE_AVAILABLE
	 } command;

 //Utility function that print a certain command
 void print_command( enum internal_command_t command ) {
	switch( command ) {
	case ALARM_ON:
		printf("ALARM_ON");
		break;
	case ALARM_OFF:
		printf("ALARM_OFF");
		break;
	case GATE_LOCK:
		printf("GATE_LOCK");
		break;
	case GATE_UNLOCK:
		printf("GATE_UNLOCK");
		break;
	case START_AUTO_OPENING:
		printf("START_AUTO_OPENING");
		break;
	case END_AUTO_OPENING:
		printf("END_AUTO_OPENING");
		break;
	case INTERNAL_TEMP:
		printf("INTERNAL_TEMP");
		break;
	case EXTERNAL_LIGHT:
		printf("EXTERNAL_LIGHT");
		break;
	case GARAGE_OPEN:
		printf("GARAGE_OPEN");
		break;
	case GARAGE_CLOSED:
		printf("GARAGE_CLOSED");
		break;
	case GARAGE_AVAILABLE:
		printf("GARAGE_AVAILABLE");
		break;
	default:
		printf("COMMAND ERROR...");
		break;
	}
	printf("\n");
 }
//Keep the last sender address of the last packet
static uint8_t last_sender[2];

//If the sender is different from Central unit, DO NOTHING!
static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];

	if( ( last_sender[0] == CU_NODE_ADDRESS_0 ) && ( last_sender[1] == CU_NODE_ADDRESS_1 ) ) {
		//printf("[door node]: broadcast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
		// Since the processes have not been declared yet, the message is sent to all processes
		process_post( NULL, message_from_central_unit, (char *)packetbuf_dataptr() );
	}
}

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	//printf("[door node]: runicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	// Since the processes have not been declared yet, the message is sent to all processes
	process_post(NULL, message_from_central_unit, (char *)packetbuf_dataptr());
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("[door node]: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("[door node]: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

//This is simply the part of code for sending a packet to Central Unit
void r_send_to_cu( void* msg ) {
	if( !runicast_is_transmitting( &runicast ) ) {
		linkaddr_t recv;
		packetbuf_copyfrom( msg, strlen( msg ) + 1 );
		recv.u8[0] = CU_NODE_ADDRESS_0;
		recv.u8[1] = CU_NODE_ADDRESS_1;

		//printf("[door node]: %u.%u: sending runicast to address %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], recv.u8[0], recv.u8[1]);
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
	} else {
		// The previous transmission has not finished yet
		printf("[door node]: It was not possible to issue the command. Try again later\n");
	}
}

//Behaviour
/*---------------------------------------------------------------------------*/
PROCESS( door_node_main_process, "Door Node Main Process");
PROCESS( door_node_alarm_blink_process, "Door Node Alarm Led Process");
PROCESS( door_node_opening_blink_process, "Door Node Opening Led Process");
PROCESS( door_node_temperature_process, "Door Node Temperature Process");
PROCESS( door_node_button_process, "Door Node Button Process");

AUTOSTART_PROCESSES( &door_node_main_process, &door_node_temperature_process, &door_node_button_process );
/*---------------------------------------------------------------------------*/

PROCESS_THREAD( door_node_main_process, ev, data )
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));

	PROCESS_BEGIN();

	static char out_msg[ MSG_LENGTH ];		// stores the message to be sent to the central unit
	static bool inserted_alarm_during_auto_opening = false;

	home_status_init( );// initializes the system status & leds

	// This customized message is declared here even if it is used in the broadcast received function. But it's ok, since the broadcast_open function
	// is called only after the custom event initialization.
	message_from_central_unit = process_alloc_event();
	alarm_blink = process_alloc_event();
	opening_blink = process_alloc_event();
	end_blinking = process_alloc_event();

	broadcast_open( &broadcast, 129, &broadcast_call );
	runicast_open( &runicast, 144, &runicast_calls );

	SENSORS_ACTIVATE( button_sensor );//Button sensor activation for SWITCH ON/OFF garden lights

	queue_init();// initialize the circular queue in charge of storing temperature values

	while( 1 ) {
		//Waiting for either a message from the central unit, a button click or for a message from another process.
		PROCESS_WAIT_EVENT();

		if( ev == message_from_central_unit ) { // Message from the central unit

			command = atoi( data );

			switch( command ) {

				case ALARM_ON: /* alarm activation command */
					if( home_status[ ALARM ] == OFF ) {
						if( home_status[ DOOR ] == AUTO_OPENING )
							inserted_alarm_during_auto_opening = true;

						store_status( inserted_alarm_during_auto_opening );//Save the current status
						home_status[ ALARM ] = ON;


						process_start( &door_node_alarm_blink_process, NULL );//Starting blinking process
					}
					break;

				case ALARM_OFF: /* alarm deactivation command */
					if( home_status[ ALARM ] == ON ) {
						home_status[ ALARM ] = OFF;
						process_exit( &door_node_alarm_blink_process );//Killing blinking process

						restore_status( );
					}
					break;

				case START_AUTO_OPENING: /* auto opening command */
					if( ( home_status[ ALARM ] == OFF ) && ( home_status[ DOOR ] == CLOSED ) ) {
						store_status( inserted_alarm_during_auto_opening );
						//printf("Previous status auto opening blinking: "); print_state( previous_status[ DOOR ]);
						home_status[ DOOR ] = AUTO_OPENING;
						process_start( &door_node_opening_blink_process, NULL );
					}
					break;

				case INTERNAL_TEMP:  /* temperature mean value command */
					if( home_status[ ALARM ] == OFF ) {
						sprintf( out_msg, "%d-%d", INTERNAL_TEMP, queue_mean_get());
						r_send_to_cu( out_msg );
					}
					break;

				default:
					printf("[door node]: COMMAND NOT VALID\n");
					break;
			}
		} else if( ev == alarm_blink ) {//Message from the alarm_blink process. We must change the leds in the alarm way.
			//It may happen that gate_node_alarm_led_process has sent the alarm_blink event just before the "deactivate alarm" was issued.
			if( home_status[ ALARM ] == ON ) {
				if( leds_get() != 0 )//Some led is ON
					leds_off( LEDS_ALL );
				else//ALL Leds are OFF
					leds_on( LEDS_ALL );
			}
		} else if( ev == opening_blink ) {//Mssage from the alarm_blink process. We must change the leds in the auto-opening way.
			if( ( home_status[ ALARM ] == OFF ) && ( home_status[ DOOR ] == AUTO_OPENING ) ) {
				//In order to have the blue led turned on the first time, turned off the second time and so on,
				//the process send us the number of occurrence of this message, and we use the occurrence number
				//in order to understand if we have to turn on or turn off the blue led.
				if( ( (int)data % 2) == 0 )
					leds_on( LEDS_BLUE );
				else
					leds_off( LEDS_BLUE );
			}
		} else if( ev == end_blinking ) {//Stopping the blinking
			if( inserted_alarm_during_auto_opening == true )//Depends from the alarm state because the alarm can be activated during auto_opening
				inserted_alarm_during_auto_opening = false;//reset the variable

			restore_status();
			//printf("Restored status a fine blinking: ");  print_state( home_status[ DOOR ]);
			sprintf( out_msg, "%d", END_AUTO_OPENING );
			r_send_to_cu( out_msg );

		} else if ( ev == button_event ) {
			 //if( ev == sensors_event && data == &button_sensor ) {//Switch ON/OFF garden lights
			if( home_status[ ALARM ] == OFF ) {// If alarm is not active
				if( home_status[ GARDEN_LIGHTS ] == OFF ) {
					home_status[ GARDEN_LIGHTS ] = ON;
					turn_on_garden_lights();
				} else {
					home_status[ GARDEN_LIGHTS ] = OFF;
					turn_off_garden_lights();
				}
			}
		}

	}
	PROCESS_END();
	return 0;
}
/*****************************************************************************************************************************************************************/
/*
 * This process, started only when the alarm is activated, is in charge of sending periodic events to the main process so that
 * the latter one knows it has to blink all leds.
 */
PROCESS_THREAD( door_node_alarm_blink_process, ev, data )
{
	PROCESS_BEGIN();
	static struct etimer blink_timer;
	etimer_set( &blink_timer, BLINKIN_PERIOD );

	while( 1 ) {

		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
			process_post( &door_node_main_process, alarm_blink, NULL );
			etimer_reset( &blink_timer );
		}
	}

	PROCESS_END();
	return 0;
}

/*****************************************************************************************************************************************************************/
/*
 * This process, started only when the automatic opening and closing of the door is issued, is in charge of waiting for the correct amount of time to elapse,
 * and then communicating the main process to blink.
 * When the blinking should be stopped, this process sends another events to the main process.
 */
PROCESS_THREAD( door_node_opening_blink_process, ev, data )
{
	PROCESS_BEGIN();

	static uint8_t silent_period_counter = 0;
	static const uint8_t SILENT_PERIOD = OPENING_PERIOD -1;
	static struct etimer blink_timer;

	etimer_set( &blink_timer, BLINKIN_PERIOD );

	//OPENING_PERIOD = 8
	//Blinking must starting ONLY after 14 seconds,that is 7 periods
	for( silent_period_counter = 0; silent_period_counter < SILENT_PERIOD; silent_period_counter++ ) {
		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
				etimer_reset( &blink_timer );
				//printf("[door node]: silent_period_counter %d\n", silent_period_counter);
		}
	}

	//After 14 secons( 7 periods ), blue led starts to blinking for 16 seconds!
	for( remained_seconds = OPENING_PERIOD; remained_seconds > 0; remained_seconds-- ) {
		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
			etimer_reset( &blink_timer );
			process_post( &door_node_main_process, opening_blink, (void*)(int)remained_seconds );
			//printf("[door node]: remained seconds %d\n", remained_seconds );
		}
	}
	process_post( &door_node_main_process, end_blinking, NULL );

	PROCESS_END();
	return 0;
}

/*****************************************************************************************************************************************************************/
/*
 * This process is in charge of sampling the temperature every 10 seconds;
 * Each sample is stored in a circular buffer (implementing a FIFO queue), where only 5 elements can be stored. So, every time a new element is put
 * into the queue, it replaces the oldest one.
 */
PROCESS_THREAD( door_node_temperature_process, ev, data )
{
	PROCESS_BEGIN();

	static struct etimer temperature_timer;
	etimer_set( &temperature_timer, SAMPLING_TEMPERATURE_PERIOD );

	while( 1 ) {

		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &temperature_timer ) ) {

			SENSORS_ACTIVATE( sht11_sensor );
			queue_insert( ( (sht11_sensor.value( SHT11_SENSOR_TEMP ) / 10 - 396 ) / 10 ) );
			SENSORS_DEACTIVATE( sht11_sensor);

			etimer_reset( &temperature_timer );
		}
	}
	PROCESS_END();
	return 0;
}
/*********************************************************************************************************************************************/
/*
 * This process sends asynchronously a button event to the Main Process every time the button is pressed
 */
PROCESS_THREAD( door_node_button_process, ev, data ) {
	PROCESS_BEGIN();

	SENSORS_ACTIVATE( button_sensor );//Button sensor activation

	while( 1 ) {

		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);//Waiting a button pressure

		process_post( &door_node_main_process, button_event, NULL );

	}//end while

	SENSORS_DEACTIVATE( button_sensor );
	PROCESS_END();
	return 0;
}


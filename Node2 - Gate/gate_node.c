/*
 * ASSUMPTIONS:
 * The initial state is: alarm OFF, gate locked, door closed, garden lights turned off, windows closed
 * Possible states are:		( D = DEAFULT )
 * 	alarm:		OFF(D) - ON
 * 	gate:		LOCKED(D) - UNLOCKED - AUTO_OPENING
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
#include "dev/light-sensor.h" //only in gate node
#include "stdbool.h"

#define MAX_RETRANSMISSIONS 			5
#define MSG_LENGTH						2
#define SAMPLING_TEMPERATURE_PERIOD		CLOCK_SECOND*10
#define BLINKIN_PERIOD					CLOCK_SECOND*2
#define OPENING_PERIOD					8

//Address
#define CU_NODE_ADDRESS_0			3
#define CU_NODE_ADDRESS_1			0

static process_event_t message_from_central_unit;
static process_event_t alarm_blink;
static process_event_t opening_blink;
static process_event_t end_blinking;

//Enumerator use to make easy the identification of position in home_status array
static enum home_t { ALARM, GATE } node;
//Array used to store the state of the system. They are used to perform operations in a consistent manner and for printing only the correct subset of the commands as available commands.
static enum status_t { ON, OFF, AUTO_OPENING, LOCKED, UNLOCKED, MOVING, OPEN, CLOSED, UNAVAILABLE } home_status[ 2 ];

//To maintain the previous status before the insertion of MODIFYING STATUS COMMANDS, g.e. ALARM, AUTO_OPENING
static enum status_t previous_status[ 2 ];

static uint8_t remained_seconds = 0;

//Set GREEN LED = ON, RED LED = OFF
void unlock_gate() {
	leds_on(  LEDS_GREEN );
	leds_off( LEDS_RED );
}
//Set GREEN LED = OFF, RED LED = ON
void lock_gate() {
	leds_off( LEDS_GREEN );
	leds_on(  LEDS_RED );
}

//Store the current state before the insertion of current command
void store_status( bool alarm ) {
	if( alarm == false ) {
		previous_status[ GATE ] = home_status [ GATE ];
	}//If the alarm is activated during opening process, the state has not be store! because previuos = auto_opening... overwriting the previous one
}


//Restore the previous state after the insertion of last command g.e. ALARM or AUTO_OPENING
void restore_status( ) {

	home_status[ GATE ] = previous_status [ GATE ];

	// leds has to return in their previous state
	if( home_status[ GATE ] == AUTO_OPENING )
		leds_on( LEDS_BLUE );
	else
		leds_off( LEDS_BLUE );

	if( home_status[ GATE ] == UNLOCKED )
		unlock_gate();
	else//Gate locked
		lock_gate();
}

//The initial state is: alarm deactivated, gate locked
void home_status_init() {
	node = ALARM;//NOTHING IMPORTANT, JUST TO MAKE HAPPY THE COMPILER BECAUSE OTHERWISE EMITS UNUSAGE WARNING
	home_status [ ALARM ] = OFF;
	home_status [ GATE ]  = LOCKED;

	lock_gate();

	previous_status[ ALARM ] =  home_status [ ALARM ];
	previous_status[ GATE ] = home_status [ GATE ];
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

//Keep the last sender address of the last packet
static uint8_t last_sender[2];

//If the sender is different from Central unit, DO NOTHING!
static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];

	if( ( last_sender[0] == CU_NODE_ADDRESS_0 ) && ( last_sender[1] == CU_NODE_ADDRESS_1 ) ) {
		//printf("[gate node]: broadcast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
		// Since the processes have not been declared yet, the message is sent to all processes
		process_post( NULL, message_from_central_unit, (char *)packetbuf_dataptr() );
	}
}

static void recv_runicast( struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno ) {
	//printf("[gate node]: runicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	// Since the processes have not been declared yet, the message is sent to all processes
	process_post( NULL, message_from_central_unit, (char *)packetbuf_dataptr() );
}

static void sent_runicast( struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions ) {
	//printf("[gate node]: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("[gate node]: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

//This is simply the part of code for sending a packet to Central Unit
void r_send_to_cu( void* msg ) {
	if( !runicast_is_transmitting( &runicast ) ) {
		linkaddr_t recv;
		packetbuf_copyfrom( msg, strlen( msg ) + 1);
		recv.u8[0] = CU_NODE_ADDRESS_0;
		recv.u8[1] = CU_NODE_ADDRESS_1;

		//printf("[gate node]: sending runicast to address %u.%u\n", recv.u8[0], recv.u8[1]);
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
	} else {
		// The previous transmission has not finished yet
		printf("[gate node]: It was not possible to issue the command. Try again later\n");
	}
}

int obtain_light( ) {

	SENSORS_ACTIVATE( light_sensor );

	int light = 10*light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/7;
	//printf("[gate node]: sampled light is %d\n", light);

	SENSORS_DEACTIVATE( light_sensor );

	return light;
}

//Behaviour
/*------------------------------------------------------------------------------------------------------------------------------------------*/
PROCESS( gate_node_main_process, "Gate Node Main Process");
PROCESS( gate_node_alarm_blink_process, "Gate Node Alarm Led Process");
PROCESS( gate_node_opening_blink_process, "Gate Node Opening Led Process");

AUTOSTART_PROCESSES( &gate_node_main_process );
/*------------------------------------------------------------------------------------------------------------------------------------------*/

PROCESS_THREAD( gate_node_main_process, ev, data )
{
	PROCESS_EXITHANDLER( broadcast_close(&broadcast) );
	PROCESS_EXITHANDLER( runicast_close(&runicast) );

	PROCESS_BEGIN();

	static char out_msg[ MSG_LENGTH ];		// stores the message to be sent to the central unit
	static int ext_light = 0;
	static bool inserted_alarm_during_auto_opening = false;

	home_status_init( );// initializes the system status & leds

	SENSORS_ACTIVATE( light_sensor );//Switch on the light sensor out of the while for the first time, to produce a read value different from 0

	// This customized message is declared here even if it is used in the broadcast received function. But it's ok, since the broadcast_open function
	// is called only after the custom event initialization.
	message_from_central_unit = process_alloc_event();
	alarm_blink = process_alloc_event();
	opening_blink = process_alloc_event();
	end_blinking = process_alloc_event();

	broadcast_open( &broadcast, 129, &broadcast_call );
	runicast_open( &runicast, 144, &runicast_calls );

	while( 1 ) {
		//Waiting for either a message from the central unit, a button click or for a message from another process.
		PROCESS_WAIT_EVENT();

		if( ev == message_from_central_unit ) { // Message from the central unit

			command = atoi( data );
			//printf("Comando ricevuto dal processo: %d, ", command);print_command( command );

			switch( command ) {

				case ALARM_ON: /* alarm activation command */
					if( home_status[ ALARM ] == OFF ) {

						if( home_status[ GATE ] == AUTO_OPENING )
							inserted_alarm_during_auto_opening = true;

						store_status( inserted_alarm_during_auto_opening );//Save the current status
						home_status[ ALARM ] = ON;

						process_start( &gate_node_alarm_blink_process, NULL );//Starting blinking process
					}
					break;

				case ALARM_OFF: /* alarm deactivation command */
					if( home_status[ ALARM ] == ON ) {
						home_status[ ALARM ] = OFF;
						process_exit( &gate_node_alarm_blink_process );//Killing blinking process

						restore_status( );
					}
					break;

				case GATE_LOCK: /* Locking command */
					if( ( home_status[ ALARM ] == OFF ) && ( home_status[ GATE ] == UNLOCKED ) ) {
						home_status[ GATE ] = LOCKED;
						lock_gate();
					}
					break;

				case GATE_UNLOCK: /*Unlocking command */
					if( ( home_status[ ALARM ] == OFF ) && ( home_status[ GATE ] == LOCKED ) ) {
						home_status[ GATE ] = UNLOCKED;
						unlock_gate();
					}
					break;

				case START_AUTO_OPENING: /* auto opening command */
					if( ( home_status[ ALARM ] == OFF ) && ( home_status[ GATE ] != AUTO_OPENING ) ) {
						store_status( inserted_alarm_during_auto_opening );
						//printf("Previous status su inizio auto_opening: ");  print_state( previous_status[ GATE ]);
						home_status[ GATE ] = AUTO_OPENING;
						process_start( &gate_node_opening_blink_process, NULL );
					}
					break;

				case EXTERNAL_LIGHT: /* auto opening command */
					if( home_status[ ALARM ] == OFF ) {
						ext_light = obtain_light();
						sprintf( out_msg, "%d-%d", EXTERNAL_LIGHT, ext_light );
						r_send_to_cu( out_msg );
					}
					break;

				default:
					printf("[gate node]: COMMAND NOT VALID\n");
					break;
			}//end switch
		} else if( ev == alarm_blink ) {//Message from the alarm_blink process. We must change the leds in the alarm way.
			// It may happen that gate_node_alarm_led_process has sent the alarm_blink event just before the "deactivate alarm" was issued.
			if( home_status[ ALARM ] == ON ) {
				if( leds_get() != 0 )//Some led is ON
					leds_off( LEDS_ALL );
				else//ALL Leds are OFF
					leds_on( LEDS_ALL );
			}

		} else if( ev == opening_blink ) {//Message from the gate_node_opening_blink process.

			if( ( home_status[ ALARM ] == OFF ) && ( home_status[ GATE ] == AUTO_OPENING ) ) {
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
				inserted_alarm_during_auto_opening = false;

			restore_status();
			//printf("Restored status a fine blinking: GATE ");print_state( home_status[ GATE ]);

			sprintf( out_msg, "%d", END_AUTO_OPENING );
			r_send_to_cu( out_msg );
		}
	}
	PROCESS_END();
	return 0;
}

/*********************************************************************************************************************************************/

/*
 * This process, started only when the alarm is activated, is in charge of sending periodic events to the main process so that
 * the latter one knows it has to blink all leds.
 */
PROCESS_THREAD( gate_node_alarm_blink_process, ev, data )
{
	PROCESS_BEGIN();
	static struct etimer blink_timer;
	etimer_set( &blink_timer, BLINKIN_PERIOD );

	while( 1 ) {

		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
			process_post( &gate_node_main_process, alarm_blink, NULL );
			etimer_reset( &blink_timer );
		}
	}

	PROCESS_END();
	return 0;
}

/*********************************************************************************************************************************************/

/*
 * This process, started only when the automatic opening and closing of the door is issued, is in charge of waiting for the correct amount
 * of time to elapse, and then communicating the main process to blink.
 * When the blinking should be stopped, this process sends another events to the main process.
 */
PROCESS_THREAD( gate_node_opening_blink_process, ev, data )
{
	PROCESS_BEGIN();

	//static uint8_t period_counter = 0;

	static struct etimer blink_timer;

	etimer_set( &blink_timer, BLINKIN_PERIOD );

	//OPENING_PERIOD = 8 --> 16 seconds
	for( remained_seconds = OPENING_PERIOD; remained_seconds > 0; remained_seconds-- ) {
		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
			//printf("[gate node]: remained second %d\n", remained_seconds);
			process_post( &gate_node_main_process, opening_blink, (void*)(int)remained_seconds );
			etimer_reset( &blink_timer );
		}
	}
	process_post( &gate_node_main_process, end_blinking, NULL );

	PROCESS_END();
	return 0;
}

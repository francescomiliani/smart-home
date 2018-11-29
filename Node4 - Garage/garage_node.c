/*
 * ASSUMPTIONS:
 * The initial state is:
 * Possible states are:		( D = DEAFULT )
 * 	alarm:		OFF(D) - ON
 * 	garage:		CLOSED(D) - OPEN - MOVING
 */

/*
 * NOTES!
 * THE GARAGE IS INDEPENDENT FROM THE ALARM!
 *
 * ATTENTION! ONLY FOR GARAGE AND NOT FOR REMOTE CONTROLLER THERE ARE 2 STATES FOR GARAGE!
 * IN REMOTE CONTROLLER THE GARAGE HAS GOT 4 POSSIBLE STATES:	UNAVAILABLE(D) - OPEN - CLOSED - MOVING
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "dev/button-sensor.h"
#include "net/rime/rime.h"
#include "string.h"
#include "dev/leds.h"
#include "dev/sht11/sht11-sensor.h"

#define MAX_RETRANSMISSIONS 5
#define MSG_LENGTH			8

#define CU_NODE_ADDRESS_0					3
#define CU_NODE_ADDRESS_1					0
#define REMOTE_CONTROLLER_NODE_ADDRESS_0	5
#define REMOTE_CONTROLLER_NODE_ADDRESS_1	0

#define BLINKING_PERIOD		CLOCK_SECOND*2
#define MOVING_PERIOD		8

static enum internal_command_t {
	ALARM_ON, ALARM_OFF,
	GATE_LOCK, GATE_UNLOCK,
	START_AUTO_OPENING, END_AUTO_OPENING,//END AUTO_OPENING is ONLY received from CU. NEVER SENT!
	INTERNAL_TEMP,
	EXTERNAL_LIGHT,
	GARAGE_OPEN, GARAGE_CLOSED, GARAGE_AVAILABLE
	} int_command;

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

static process_event_t message;
static process_event_t alarm_blink;
static process_event_t moving_blinking;
static process_event_t blinking_end;

//Keep the last sender address of the last packet
static uint8_t last_sender[2];

//Definition of the receiving & sending function
static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];
	//printf("[garage node]: broadcast message received from %d.%d: '%s'\n", last_sender[0], last_sender[1], (char *)packetbuf_dataptr());

	// Since the processes have not been declared yet, the message is sent to all processes
	process_post( NULL, message, (char *)packetbuf_dataptr() );

}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx){ //if status == 0, it's all OK! else some error is occurs
  //printf("[garage node]: broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static void recv_runicast( struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno ) {
	//printf("[garage node]: runicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	//Since the processes have not been declared yet, the message is sent to all processes
	process_post(NULL, message, (char *)packetbuf_dataptr());
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];
}

static void sent_runicast( struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions ) {
	//printf("[garage node]: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast( struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions ) {
	//printf("[garage node]: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent};
static struct broadcast_conn broadcast;
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

//Enumerator use to make easy the identification of position in home_status array
static enum home_t { ALARM, GARAGE } node;

//Array used to store the state of the system. They are used to perform operations in a consistent manner and for printing only the correct subset of the commands as available commands.
static enum status_t { ON, OFF, AUTO_OPENING, LOCKED, UNLOCKED, MOVING, OPEN, CLOSED, UNAVAILABLE } home_status[ 2 ];
static enum status_t previous_status[ 2 ];

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

//Set GREEN LED = ON, RED LED = OFF
void open_garage() {
	leds_on(  LEDS_GREEN );
	leds_off( LEDS_RED );
}

//Set GREEN LED = OFF, RED LED = ON
void close_garage() {
	leds_off( LEDS_GREEN );
	leds_on(  LEDS_RED );
}

void home_status_init() {
	node = ALARM;//NOTHING IMPORTANT, JUST TO MAKE HAPPY THE COMPILER BECAUSE OTHERWISE EMITS UNUSAGE WARNING
	home_status [ ALARM ] = OFF;
	home_status [ GARAGE ] = CLOSED;
	previous_status[ GARAGE ] = CLOSED;

	close_garage();//Modifying the leds
}

//Store the current state before the insertion of current command
//ONLY FOR GARAGE NODE, STORE STATUS NOT RECEIVE AS PARAMETER THE ALARM STATE because normally is INDEPENDENT, but if the alarm is activated during MOVING
//some problem occurs. So store status is necessary
void store_status( bool alarm ) {
	if( alarm == true )
		previous_status[ GARAGE ] =  home_status [ GARAGE ];
}

//Restore the previous state ( including the leds ) after the insertion of last command g.e. ALARM or AUTO_OPENING
void restore_status( ) {

	home_status[ ALARM ] =  previous_status [ GARAGE ];

	//if( alarm == true )
	if( home_status[ GARAGE ] == OPEN ) {
		open_garage();
		leds_off( LEDS_BLUE );
	}
	else if( home_status[ GARAGE ] == CLOSED ) {
		close_garage();
		leds_off( LEDS_BLUE);
	}
	else {//Moving
		//printf("[garage node]: Moving al restoring\n");
		leds_on( LEDS_BLUE );
		if( previous_status[ GARAGE ] == OPEN )//Control previuos and NOT home because home is moving... does not contain useful information
			open_garage();
		else
			close_garage();
	}
}

void r_send( void* msg, int len, int rime_addr_0, int rime_addr_1 ) {
	if( !runicast_is_transmitting( &runicast ) ) {
		linkaddr_t recv;
		packetbuf_copyfrom(msg, len);
		recv.u8[0] = rime_addr_0;
		recv.u8[1] = rime_addr_1;
		//printf("[garage node]: %u.%u: sending runicast to address %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], recv.u8[0], recv.u8[1]);
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
	} else {
		// The previous transmission has not finished yet
		printf("[garage node]: It was not possible to issue the command. Try again later\n");
	}
}
/*---------------------------------------------------------------------------*/
PROCESS( garage_node_main_process, "Garage Node Main Process");
PROCESS( garage_node_alarm_blink_process, "Garage Node Alarm Led Process");
PROCESS( garage_node_timer_process, "Garage Node Timer Process");

AUTOSTART_PROCESSES( &garage_node_main_process );
/*---------------------------------------------------------------------------*/

PROCESS_THREAD( garage_node_main_process, ev, data )
{
	PROCESS_EXITHANDLER( broadcast_close( &broadcast ) );
	PROCESS_EXITHANDLER( runicast_close( &runicast ) );

	PROCESS_BEGIN();

	static char out_msg[ MSG_LENGTH ];
	static bool inserted_alarm_during_auto_opening = false;

	home_status_init();

	//This customized message is declared here even if it is used in the broadcast received function.
	//But it's ok, since the broadcast_open function is called only after the custom event initialization.
	message = process_alloc_event();
	alarm_blink = process_alloc_event();
	moving_blinking = process_alloc_event();
	blinking_end = process_alloc_event();

	broadcast_open( &broadcast, 129, &broadcast_call );
	runicast_open( &runicast, 144, &runicast_calls );


	while( 1 ) {

		// We wait for either a message from the central unit, remote controller, or for a message from another process.
		PROCESS_WAIT_EVENT();

		if( ev == message ) { // Message from the central unit

			int_command = atoi( data );

			if( (last_sender[0] == CU_NODE_ADDRESS_0 ) && (last_sender[1] == CU_NODE_ADDRESS_1 ) ) {
				//printf( "[garage node]: Messaggio dalla central unit\n" );
				switch( int_command ) {

					case ALARM_ON:/* alarm activation command */
						if( home_status[ ALARM ] == OFF ) {

							//printf("[garage node]: Previous status INIZIO allarme: ");  print_state( previous_status[ GARAGE ] );
							home_status[ ALARM ] = ON;
							if( home_status[ GARAGE ] == MOVING )//In case that alarm insertion during moving of garage
								inserted_alarm_during_auto_opening = true;
							//store_status();//Not necessary because garage is alarm independent
							process_start( &garage_node_alarm_blink_process, NULL );//Starting blinking process
						}
						break;

					case ALARM_OFF:/* alarm deactivation command */
						if( home_status[ ALARM ] == ON ) {

							//printf("[garage node]: Previous status FINE allarme: ");  print_state( previous_status[ GARAGE ]);
							restore_status( );
							home_status[ ALARM ] = OFF;

							process_exit( &garage_node_alarm_blink_process );//Killing blinking process
						}
						break;

					case GARAGE_OPEN:/* Opening of the garage command */
						if( (home_status[ ALARM ] == OFF)  && (home_status[ GARAGE ] != MOVING) ) {
							previous_status[ GARAGE ] = CLOSED;
							home_status[ GARAGE ] = MOVING;

							sprintf( out_msg, "%d-%d", home_status[ GARAGE ], previous_status[ GARAGE ] );
							r_send( out_msg, strlen(out_msg)+1, REMOTE_CONTROLLER_NODE_ADDRESS_0, REMOTE_CONTROLLER_NODE_ADDRESS_1 );

							process_start( &garage_node_timer_process, NULL );
						}
						break;

					case GARAGE_CLOSED:/* Closing of the garage command */
						if( (home_status[ ALARM ] == OFF)  && (home_status[ GARAGE ] != MOVING) ) {
							previous_status[ GARAGE ] = OPEN;
							home_status[ GARAGE ] = MOVING;

							sprintf( out_msg, "%d-%d", home_status[ GARAGE ], previous_status[ GARAGE ] );
							r_send( out_msg, strlen(out_msg)+1, REMOTE_CONTROLLER_NODE_ADDRESS_0, REMOTE_CONTROLLER_NODE_ADDRESS_1 );

							process_start( &garage_node_timer_process, NULL );
						}
					default:
						printf( "[garage node]: ERROR! COMMAND NOT VALID\n" );
						break;
				}//end switch
			}//END IF CENTRAL UNIT MESSAGE
			else if( (last_sender[0] == REMOTE_CONTROLLER_NODE_ADDRESS_0 ) && (last_sender[1] == REMOTE_CONTROLLER_NODE_ADDRESS_1 ) ) {
				//printf( "[garage node]: Messaggio dal remote controller\n" );
				if( int_command == GARAGE_AVAILABLE ) {

					if( home_status[ GARAGE ] != MOVING )//otherwise the previous status will be lost
						previous_status[ GARAGE ] = home_status[ GARAGE ];

					home_status[ GARAGE ] = MOVING;
					//Sending confirmation in broadcast to remote controller & central unit. Others node will discard the packet
					sprintf( out_msg, "%d-%d", home_status[ GARAGE ], previous_status[ GARAGE ] );//current status ( can be MOVING ), & previous status different from MOVING

					packetbuf_copyfrom( out_msg, strlen( out_msg )+1 );
					broadcast_send( &broadcast );

					//Before killing the timer process to restart it subsequently, if the process it is not active, NOTHING HAPPEN
					process_exit( &garage_node_timer_process );//Killing timer process

					process_start( &garage_node_timer_process, NULL );

				} else
						printf( "[garage node]: ERROR! COMMAND NOT VALID\n" );
			}//end if remote controller

		}//end if message
		else if( ev == alarm_blink ) {

			if( home_status[ ALARM ] == ON ) {
				if( leds_get( ) != 0 )//Return an array byte where each bit represent the status of a single led
					leds_off( LEDS_ALL );
				else
					leds_on( LEDS_ALL );
			}
		}else if( ev == moving_blinking ) {
			//printf("[garage node]: Moving blinking: %s, period = %d\n", ( (int)data % 2) == 0 ? "ON":"OFF", (int)data );
			if( ( (int)data % 2) == 0 )
				leds_on( LEDS_BLUE );
			else
				leds_off( LEDS_BLUE );

		}else if( ev == blinking_end ) {//Stop blinking

			//printf("[garage node]: Previous status fine opening: ");  print_state( previous_status[ GARAGE ]);
			if(inserted_alarm_during_auto_opening == true )
				inserted_alarm_during_auto_opening = false;

			if( previous_status[ GARAGE ] == CLOSED ) {
				home_status[ GARAGE ] = OPEN;
				open_garage();
			}else { //previous_garage_status == OPEN
				home_status[ GARAGE ] = CLOSED;
				close_garage();
			}
			leds_off( LEDS_BLUE );
			//printf("[garage node]: Home status fine opening: ");  print_state( home_status[ GARAGE ]);
			sprintf( out_msg, "%d", home_status[ GARAGE ]);

			packetbuf_copyfrom( out_msg, strlen( out_msg )+1 );
			broadcast_send( &broadcast );

		}//end blinking end
	}//end while
	PROCESS_END();
	return 0;
}
/*******************************************************************************************************************************************/
/*
 * This process, started only when the alarm is activated, is in charge of sending periodic events to the main process so that
 * the latter one knows it has to blink all leds.
 */
PROCESS_THREAD( garage_node_alarm_blink_process, ev, data )
{
	PROCESS_BEGIN();

	static struct etimer blink_timer;
	etimer_set( &blink_timer, BLINKING_PERIOD );

	while( 1 ) {

		PROCESS_WAIT_EVENT();

		if(ev == PROCESS_EVENT_TIMER && etimer_expired( &blink_timer ) ) {
			process_post( &garage_node_main_process, alarm_blink, NULL );
			etimer_reset( &blink_timer );
		}
	}
	PROCESS_END();
	return 0;
}

/****************************************************************************************************************************************/
/*
 * This process, started only when the automatic opening OR closing of the garage is issued, is in charge of waiting for the correct amount
 * of time to elapse, and then communicating the main process to blink.
 * When the blinking should be stopped, this process sends another events to the main process.
 */
PROCESS_THREAD( garage_node_timer_process, ev, data )
{
	PROCESS_BEGIN();

	static uint8_t period_counter = 0;
	static struct etimer timer;

	etimer_set( &timer, BLINKING_PERIOD );

	//OPENING_PERIOD = 8 --> 16 seconds
	for( period_counter = 0; period_counter < MOVING_PERIOD; period_counter++ ) {
		PROCESS_WAIT_EVENT();

		if( ev == PROCESS_EVENT_TIMER && etimer_expired( &timer ) ) {
			process_post( &garage_node_main_process, moving_blinking, (void*)(int)period_counter );
			etimer_reset( &timer );
		}
	}

	process_post( &garage_node_main_process, blinking_end, NULL );

	PROCESS_END();
	return 0;
}

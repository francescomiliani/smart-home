/*
 * ASSUMPTIONS:
 * The initial state is: alarm OFF, gate locked, door closed, garden lights turned off, windows closed
 * Possible states are:		( D = DEAFULT )
 * 	alarm:		OFF(D) - ON
 *  garage:		UNAVAILABLE(D) - OPEN - CLOSED - MOVING
 */

/*
 * NOTES:
 * This node is different from the other one because communicate by means of MULTI HOP
 * Send his packets ONLY to GARAGE NODE
 * This node sends a packet to the Garage Node to open it or close it.
 * If the garage is in the transmission range, the communication will be successful, else will fail.
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "stdlib.h"
#include "net/rime/rime.h"
#include "dev/button-sensor.h"
#include "string.h"
#include "dev/leds.h"

//Working Constant

#define MAX_RETRANSMISSIONS 5
#define MSG_LENGTH			1

#define GARAGE_NODE_ADDRESS_0		4
#define GARAGE_NODE_ADDRESS_1		0

static enum internal_command_t {
	ALARM_ON, ALARM_OFF,
	GATE_LOCK, GATE_UNLOCK,
	START_AUTO_OPENING, END_AUTO_OPENING,//END AUTO_OPENING is ONLY received from CU. NEVER SENT!
	INTERNAL_TEMP,
	EXTERNAL_LIGHT,
	GARAGE_OPEN, GARAGE_CLOSED, GARAGE_AVAILABLE
	} int_command;

//Array used to store the state of the garage
static enum status_t { ON, OFF, AUTO_OPENING, LOCKED, UNLOCKED, MOVING, OPEN, CLOSED, UNAVAILABLE } garage_status;
static enum status_t previous_status;

static process_event_t message_from_garage_node;
static process_event_t button_event;

//Keep the last sender address of the last packet
static uint8_t last_sender[2];

void set_garage( enum status_t value ) {
	switch( value ) {
		case UNAVAILABLE:
			leds_off(  LEDS_GREEN );
			leds_off( LEDS_RED );
			leds_off( LEDS_BLUE );
			break;
		case OPEN:
			leds_on(  LEDS_GREEN );
			leds_off( LEDS_RED );
			leds_off( LEDS_BLUE );
			break;
		case CLOSED:
			leds_off(  LEDS_GREEN );
			leds_on( LEDS_RED );
			leds_off( LEDS_BLUE );
			break;
		case MOVING:
			leds_off(  LEDS_GREEN );
			leds_off( LEDS_RED );
			leds_on( LEDS_BLUE );
			break;
		default:
			break;
	}
}

/*---------  PROCESS -----------------------------------------------------------------------------------------------------------------------*/
PROCESS( remote_controller_node_button_process, "Remote Controller Button Process" );
PROCESS( remote_controller_node_main_process, "Remote Controller Button Main Process");

/*-------------------------------------------------------------------------------------------------------------------------------------*/

//If the sender is different from Garage node, DO NOTHING!
static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {

	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];
	if( ( last_sender[0] == GARAGE_NODE_ADDRESS_0 ) && ( last_sender[1] == GARAGE_NODE_ADDRESS_1 ) ) {
		//printf("[remote controller node]: broadcast message received from %d.%d: '%s'\n", last_sender[0], last_sender[1], (char *)packetbuf_dataptr());
		// Since the processes have not been declared yet, the message is sent to all processes
		process_post( NULL, message_from_garage_node, (char *)packetbuf_dataptr() );
	}
}

static void recv_runicast( struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno ) {
	//printf("[remote controller node]: runicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	//Since the processes have not been declared yet, the message is sent to all processes
	process_post( NULL, message_from_garage_node, (char *)packetbuf_dataptr() );
}

static void sent_runicast( struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions ) {
	//printf("[remote controller node]: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast( struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions ) {
	//printf("[remote controller node]: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);

	garage_status = UNAVAILABLE;
	set_garage( UNAVAILABLE );
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

//The initial state is: alarm deactivated, gate locked, door off, windows closed, garden lights turned off.
void home_status_init() {
	garage_status = UNAVAILABLE;
}

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
	default:
		printf("STATUS ERROR...");
		break;
	}
	printf("\n");

}

//This node is different from the other one because communicate by means of MULTI HOP
//Send his packets ONLY to GARAGE NODE
void r_send_to_garage_node( void* msg ) {
	if( !runicast_is_transmitting( &runicast ) ) {
		linkaddr_t recv;
		packetbuf_copyfrom( msg, strlen( msg ) + 1 );
		recv.u8[0] = GARAGE_NODE_ADDRESS_0;
		recv.u8[1] = GARAGE_NODE_ADDRESS_1;
		// printf("remote controller node: %u.%u: sending runicast to address %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], recv.u8[0], recv.u8[1]);
		runicast_send( &runicast, &recv, MAX_RETRANSMISSIONS );
	} else {
		// The previous transmission has not finished yet
		printf("[remote controller node]: It was not possible to issue the command. Try again later\n");
	}
}


/*---------------------------------------------------------------------------------------------------------------------------------------*/
AUTOSTART_PROCESSES( &remote_controller_node_main_process, &remote_controller_node_button_process );
/*---------------------------------------------------------------------------------------------------------------------------------------*/
PROCESS_THREAD( remote_controller_node_button_process, ev, data ) {
	PROCESS_BEGIN();

	SENSORS_ACTIVATE( button_sensor );//Button sensor activation

	while( 1 ) {

		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);//Waiting a button pressure

		process_post( &remote_controller_node_main_process, button_event, NULL );

	}//end while

	SENSORS_DEACTIVATE( button_sensor );
	PROCESS_END();
	return 0;
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
PROCESS_THREAD( remote_controller_node_main_process, ev, data ) {

	PROCESS_EXITHANDLER( broadcast_close( &broadcast ) );
	PROCESS_EXITHANDLER( runicast_close( &runicast ) );

	PROCESS_BEGIN();

	static char out_msg[ MSG_LENGTH ];		// stores the message to be sent to the central unit
	static char in_msg[ MSG_LENGTH ];		// stores the message to be sent to the central unit
	static char *current_home_status = 0;
	static char *previous_home_status = 0;
	static enum status_t chstatus;
	//static struct etimer sending_timer;
	//etimer_set( &sending_timer, SENDING_PERIOD );

	home_status_init();

	//This customized message is declared here even if it is used in the broadcast received function.
	//But it's ok, since the broadcast_open function is called only after the custom event initialization.
	message_from_garage_node = process_alloc_event();
	button_event = process_alloc_event();

	broadcast_open( &broadcast, 129, &broadcast_call );
	runicast_open( &runicast, 144, &runicast_calls );

	while( 1 ) {//Waiting for either a message from the Garage Node or a button click
		PROCESS_WAIT_EVENT();

		if( ev == message_from_garage_node ) { // Message from the central unit
			strcpy( in_msg, (char*)data );//Message format:	command_code-[value]     value not always is needed

			current_home_status = strtok( in_msg, "-" );
			previous_home_status = strtok( current_home_status, "-" );//Would be previous status
			chstatus = atoi( current_home_status );//Convert in integer
			//printf( "[remote controller node]: current_home_status: ");print_state( atoi(current_home_status) );
			//printf("          previous home status: " );print_state( atoi( previous_home_status ) );
			//printf(",   %s\n",  previous_home_status);
			switch( chstatus ) {

				case OPEN:
					//printf( "[remote controller node]: Garage open\n" );
					garage_status = OPEN;
					previous_status = CLOSED;
					set_garage( CLOSED );//To indicate that the remote controller can CLOSE the garage
					break;

				case CLOSED:
					//printf( "[remote controller node]: Garage closed\n" );
					previous_status = OPEN;
					garage_status = CLOSED;
					set_garage( OPEN );
					break;

				case MOVING:
					//printf( "[remote controller node]: Garage is moving\n" );
					previous_status = atoi( previous_home_status );
					garage_status = MOVING;
					set_garage( MOVING );//To indicate that the remote controller can CLOSE the garage
					break;

				default:
					printf( "[remote controller node]: ERROR! COMMAND NOT VALID\n" );
					break;
			}

		}else if( ev == button_event ) {//Button event
			printf("[remote controller node]: Pressed remote controller\n");

			if( garage_status == MOVING ) {
				printf( "[remote controller node]: GARAGE IS MOVING! PLEASE WAITING FOR END OF THE MOVING\n" );
			}else {
				int_command = GARAGE_AVAILABLE;
				sprintf( out_msg, "%d", int_command );
				r_send_to_garage_node( out_msg );
			}
		}
	}
	PROCESS_END();
	return 0;
}

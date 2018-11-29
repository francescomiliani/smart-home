/*
 * ASSUMPTIONS:
 * The initial state is: alarm OFF, gate locked, door closed, garden lights turned off, windows closed
 * Possible states are:		( D = DEAFULT )
 * 	alarm:				OFF(D) - ON
 * 	gate:				LOCKED(D) - UNLOCKED - AUTO_OPENING
 * 	door:				CLOSED(D) - AUTO_OPENING
 * 	garden lights:		OFF(D) - ON
 * 	garage:				CLOSED(D) - OPEN - MOVING
 */


#include "contiki.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "dev/button-sensor.h"
#include "net/rime/rime.h"
#include "string.h"
#include "dev/serial-line.h"
#include "stdlib.h"
#include "stdbool.h"

#define MAX_COMMAND_ALLOWED 6
#define MAX_RETRANSMISSIONS 5
#define MSG_LENGTH			8
#define NODE_NUMBER			5

//Timing Constants
#define BUTTON_PERIOD		CLOCK_SECOND*4
//Addresses
#define DOOR_NODE_ADDRESS_0					1
#define DOOR_NODE_ADDRESS_1					0
#define GATE_NODE_ADDRESS_0					2
#define GATE_NODE_ADDRESS_1					0
#define CU_NODE_ADDRESS_0					3
#define CU_NODE_ADDRESS_1					0
#define GARAGE_NODE_ADDRESS_0				4
#define GARAGE_NODE_ADDRESS_1				0


//Event used by the button process to communicate to the main process that a valid command has been issued (the button has been pressed a valid number of times
static process_event_t user_command_event;

//Event used by the communication callback methods to forward the content of a sensor-sent message to the main process.
static process_event_t sensor_message_event;

//Enumerator use to make easy the identification of position in home_status array
static enum home_t { ALARM, GATE, DOOR, GARDEN_LIGHTS, GARAGE } node;

//Array used to store the state of the system. They are used to perform operations in a consistent manner and for printing only the correct subset of the commands as available commands.
static enum status_t { ON, OFF, AUTO_OPENING, LOCKED, UNLOCKED, MOVING, OPEN, CLOSED, UNAVAILABLE } home_status[ NODE_NUMBER ];

//To maintain the previous status before the insertion of MODIFYING STATUS COMMANDS, g.e. ALARM, AUTO_OPENING
static enum status_t previous_status[ NODE_NUMBER ];

/*
 * Let's use 2 different kind of commands: user and internal.
 * The user ones are insert in from the User, the internal ones are used by the sensor node for all operations
 */
static enum user_command_t { ALARM_C, GATE_C, DOOR_C, INTERNAL_TEMP_C, GARDEN_LIGHTS_C, GARAGE_C } user_command;

/**
 * Internal commands for convenience and internal program usage
 */
static enum internal_command_t {
	ALARM_ON, ALARM_OFF,
	GATE_LOCK, GATE_UNLOCK,
	START_AUTO_OPENING, END_AUTO_OPENING,//END AUTO_OPENING is ONLY received from CU. NEVER SENT!
	INTERNAL_TEMP,
	EXTERNAL_LIGHT,
	GARAGE_OPEN, GARAGE_CLOSED, GARAGE_AVAILABLE
	} out_command;

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

//Utility function that print a certain command
void print_user_command( enum user_command_t command ) {
	switch( command ) {
	case ALARM_C:
		printf("ALARM_C");
		break;
	case GATE_C:
		printf("GATE_C");
		break;
	case DOOR_C:
		printf("DOOR_C");
		break;
	case INTERNAL_TEMP_C:
		printf("INTERNAL_TEMP_C");
		break;
	case GARDEN_LIGHTS_C:
		printf("GARDEN_LIGHTS_C");
		break;
	case GARAGE_C:
		printf("GARAGE_C");
		break;
	default:
		printf("COMMAND ERROR...");
		break;
	}
	printf("\n");
}

//Keep the last sender address of the last packet
static uint8_t last_sender[2];

//The initial state is: alarm deactivated, gate locked, door off, windows closed, garden lights turned off.
void home_status_init() {
	node = ALARM;//NOTHING IMPORTANT, JUST TO MAKE HAPPY THE COMPILER BECAUSE OTHERWISE EMITS UNUSAGE WARNING
	home_status [ ALARM ] = 		OFF;
	home_status [ GATE ] = 			LOCKED;
	home_status [ DOOR ] = 			CLOSED;
	home_status [ GARDEN_LIGHTS ] = OFF;
	home_status [ GARAGE ] = 		CLOSED;

	previous_status[ ALARM ] =  home_status [ ALARM ];
	previous_status[ GATE ] = home_status [ GATE ];
	previous_status[ DOOR ] = home_status [ DOOR ];
	previous_status[ GARDEN_LIGHTS ] = home_status [ GARDEN_LIGHTS ];
	previous_status[ GARAGE ] = home_status [ GARAGE ];
}

//Definition of the receiving & sending function
static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {
	//printf("[central unit]: broadcast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];

	process_post( NULL, sensor_message_event, (char*)packetbuf_dataptr() );
}

static void broadcast_sent( struct broadcast_conn *c, int status, int num_tx){ //if status == 0, it's all OK! else some error is occurs
  //printf("[central unit]: broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static void recv_runicast( struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno ) {
	//printf("[central unit]: runicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	process_post(NULL, sensor_message_event, (char*)packetbuf_dataptr());
	last_sender[0] = (uint8_t)from->u8[0];
	last_sender[1] = (uint8_t)from->u8[1];
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("[central_unit]: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("[central unit]: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent};
static struct broadcast_conn broadcast;
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

void r_send( void* msg, int len, int rime_addr_0, int rime_addr_1 ){
	if(!runicast_is_transmitting(&runicast)) {
		linkaddr_t recv;
		packetbuf_copyfrom(msg, len);
		recv.u8[0] = rime_addr_0;
		recv.u8[1] = rime_addr_1;
		//printf( "[central unit]: %u.%u: sending runicast to address %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], recv.u8[0], recv.u8[1]);
		runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS );
	} else {
		// The previous transmission has not finished yet
		printf( "[central unit]: It was not possible to issue the command. Try again later\n" );
	}
}

//Utility function that print the current status
void print_home_status() {
	printf( "Current HOME STATUS\n"
			"\tALARM:\t%s\n\tGATE:\t%s\n\tDOOR:\t%s\n\tGARDEN_LIGHTS:\t%s\n\tGARAGE:\t%s\n",
		home_status[ ALARM ] == ON ? "ON" : "OFF",
		home_status[ GATE ] == AUTO_OPENING ? "AUTO_OPENING" : home_status[ GATE ] == LOCKED ? "LOCKED" : "UNLOCKED",
		home_status[ DOOR ] == AUTO_OPENING ? "AUTO OPENING" : "CLOSED",
		home_status[ GARDEN_LIGHTS ] == ON  ? "ON" : "OFF",
		home_status[ GARAGE ] == OPEN ? "OPEN" : home_status[ GARAGE ] == CLOSED ? "CLOSED" : "MOVING" );
	printf( "\n\n" );

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

//Store the current state before the insertion of current command
void store_status( bool alarm ) {
	if( alarm == false ) {
		previous_status[ GATE ] = 			home_status [ GATE ];
		previous_status[ DOOR ] = 			home_status [ DOOR ];
		previous_status[ GARDEN_LIGHTS ] =  home_status [ GARDEN_LIGHTS ];
		previous_status[ GARAGE ] = 		home_status [ GARAGE ];
	}
	//If the alarm is activated during opening process, the state has not be store! because previuos = auto_opening... overwriting the previous one
}

//Restore the previous state after the insertion of last command g.e. ALARM or AUTO_OPENING
void restore_status( ) {
	home_status[ GATE ] = previous_status [ GATE ];
	home_status[ DOOR ] = previous_status [ DOOR ];
	home_status[ GARDEN_LIGHTS ] = previous_status [ GARDEN_LIGHTS ];
	home_status[ GARAGE ] = previous_status [ GARAGE ];
}

/*
 * Prints the available commands in the moments it is called.
 * The list of available commands depend on the value of the home_status variable, used to store the system state.
 */
void show_available_commands(){
	// To avoid concurrency problems (e.g. home_status changes value while the comparisons are performed), his initial value is used.
	static enum status_t current_status[ NODE_NUMBER ];
	current_status[ ALARM ] =  home_status [ ALARM ];
	current_status[ GATE ] = home_status [ GATE ];
	current_status[ DOOR ] = home_status [ DOOR ];
	current_status[ GARDEN_LIGHTS ] = home_status [ GARDEN_LIGHTS ];
	current_status[ GARAGE ] = home_status [ GARAGE ];

	printf("\nAvailable commands are:\n"
			"1. %s the alarm signal\n", current_status[ ALARM ] == ON ? "Deactive" : "Active" );
	if( current_status[ ALARM ] == OFF ) { // Alarm active: only "deactive alarm" command is available.
		if( current_status[ GATE ] != AUTO_OPENING )
			printf( "2. %s the gate\n", (current_status[ GATE ] == UNLOCKED ) && ( home_status[ GATE ] == UNLOCKED ) ? "Lock" : "Unlock");
		if( (current_status[ DOOR ] != AUTO_OPENING) && (current_status[ GATE ] != AUTO_OPENING) )
			printf( "3. Open (and close) the door and the gate\n" );
		// Automatic opening and closing is active: you cannot directly lock/unlock the gate
		//Others commands can be done
		printf(	"4. Average of the last 5 internal temperature values\n"
				"5. External light value\n" );
		if( current_status[ GARAGE ] != MOVING )
			printf( "6. %s the garage\n", current_status[ GARAGE ] == CLOSED ? "Open" : "Close" );
	}//else alarm
}
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
PROCESS( cu_button_process, "Central Unit Button Process" );
PROCESS( cu_main_process, 	"Central Unit Main Process" );

AUTOSTART_PROCESSES( &cu_button_process, &cu_main_process );
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
PROCESS_THREAD( cu_button_process, ev, data )
{
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));

	PROCESS_BEGIN();

	static uint8_t ret;							// stores the return value when needed
	static uint8_t button_count = 0;		// stores the number of button clicks
	//static char button_msg[ 8 ];
	static struct etimer button_timer;		// timer for waiting for a further button click after the first

	home_status_init();//Home status initialization

	//Allocation of the custom events
	user_command_event = process_alloc_event();
	sensor_message_event = process_alloc_event();

	broadcast_open( &broadcast, 129, &broadcast_call );//Opening broadcast & unicast connections
	runicast_open( &runicast, 144, &runicast_calls );

	SENSORS_ACTIVATE( button_sensor );//Button sensor activation

	show_available_commands();

	while( 1 ) { // Waiting for the first button click

		PROCESS_WAIT_EVENT();

		//Check which is the event
		if( ev == sensors_event && data == &button_sensor ) {// Button
			printf("[central unit]: Button pressed --> button count: %d\n", button_count+1 );
			if( button_count == 0 ) //Set for the first time the countdown
				etimer_set( &button_timer, BUTTON_PERIOD );
			else
				etimer_restart( &button_timer );//Each time the button is pressed, restart the timer
			button_count++;

			if( button_count > MAX_COMMAND_ALLOWED ) {
			// The button has been pressed more than MAX_COMMAND_ALLOWED times, thus the timer must be stopped (the counter is reset in the main loop)
				printf("[central unit]: COMMAND NOT VALID\n");

				button_count = 0;//Reset button count
				etimer_stop( &button_timer );
				show_available_commands();
				//break;

			} else // The number of button clicks is still valid. Wait again.
				etimer_restart( &button_timer );

		} else if( ev == PROCESS_EVENT_TIMER && etimer_expired( &button_timer ) ) {// Timer elapsed. The number of button clicks can be assumed to be valid.
			//The value is sent to the main process for further elaboration
			//printf("[central unit]: Timer for pressing button is expired!\n Button value: %d\n", button_count);
			//sprintf()
			ret = process_post( &cu_main_process, user_command_event, (void*)(int)button_count );

			button_count = 0;//Reset button count
			if (ret != 0) {
				// Event queue full
				printf("[central unit]: It was not possible to issue the command. Try again later\n");
				show_available_commands();
			}
			//break;
		}
	}//end while

	//SENSORS_DEACTIVATE( button_sensor );
	PROCESS_END();
	return 0;
}

/*********************************************************************************************************************************************/

PROCESS_THREAD( cu_main_process, ev, data )
{
	PROCESS_BEGIN();

	static char in_msg[ MSG_LENGTH ];
	static char out_msg[ MSG_LENGTH ];

	static char *command;
	static char *value;
	static enum status_t temp_status;
	static int i_command;
	static bool inserted_alarm_during_auto_opening = false;

	while( 1 ) {//Waiting for either: a command (the button has been clicked some times); a message from a sensor node;

		PROCESS_WAIT_EVENT();

		if( ev == user_command_event ) { // An user command has been received

			user_command = (enum user_command_t)data -1;//-1 because enumerator starts from 0, whereas button pressures from 1
			//printf("Comando ricevuto dal processo: %d, ", user_command);print_user_command( user_command );
			switch( user_command ) {

				case ALARM_C: // Activate/deactivate alarm command, it is always possible to issue it. It's the ONLY command send in broadcast

					if ( home_status[ ALARM ] == OFF ) { // The alarm is off, it has to be turned on.
						if( home_status[ GATE ] == AUTO_OPENING || home_status[ DOOR ] == AUTO_OPENING )
							inserted_alarm_during_auto_opening = true;

						store_status( inserted_alarm_during_auto_opening ); // Save the current status
						home_status[ ALARM ] = ON;

						out_command = ALARM_ON;
					} else { // The alarm is on, it has to be turned off.
						restore_status( );//Restore the previous state

						home_status[ ALARM ] = OFF;
						out_command = ALARM_OFF;
					}
					sprintf( out_msg, "%d", out_command );
					packetbuf_copyfrom( out_msg, strlen( out_msg)+1 );//1 = 1 Byte

					broadcast_send( &broadcast );

					show_available_commands();
					break;

				case GATE_C:// Lock/unlock command. It is possible to issue it only if
					// 1.the alarm is deactivated, 2.the automatic opening-closing procedure is not active.
					if ( ( home_status[ ALARM ] == ON ) || ( home_status[ GATE ] == AUTO_OPENING ) ) {
						if( home_status[ ALARM ] == ON )
							printf("[central unit]: ALARM ACTIVE! COMMAND NOT VALID: DEACTIVATE ALARM BEFORE\n");
						else
							printf("[central unit]: AUTO OPENING IN PROGRESS! WAIT THE END BEFORE\n");
					} else {// It is possible to issue the command
						 previous_status[ GATE ] = home_status[ GATE];
						//print_state( home_status[ DOOR ]);
						if ( home_status[ GATE ] == LOCKED ){ // the gate is locked, thus it has to be unlocked
							home_status[ GATE ] = UNLOCKED;
							out_command = GATE_UNLOCK;
						} else { // the gate is unlocked, thus it has to be locked
							home_status[ GATE ] = LOCKED;
							out_command = GATE_LOCK;
						}

						sprintf( out_msg, "%d", out_command );
						r_send( out_msg, strlen( out_msg)+1, GATE_NODE_ADDRESS_0, GATE_NODE_ADDRESS_1 );
					}
					show_available_commands();
					break;

				case DOOR_C:
					// Auto opening and closing of gate and door command. It is possible to issue it only if
					// 1.the alarm is deactivated, 2.the automatic opening-closing procedure is not already active.
					if ( (home_status[ ALARM ] == ON) || ( home_status[ DOOR ] == AUTO_OPENING ) || ( home_status[ GATE ] == AUTO_OPENING )) {
						if( home_status[ ALARM ] == ON )
								printf("[central unit]: ALARM ACTIVE! COMMAND NOT VALID: DEACTIVATE ALARM BEFORE\n");
						else
							printf("[central unit]: AUTO OPENING IN PROGRESS! WAIT THE END BEFORE\n");
					} else {// It is possible to issue the command.

						//printf("Home state a inizio blinking: GATE : "); print_state( home_status[ GATE ] );
						//printf("DOOR: "); print_state( home_status[ DOOR ] );
						previous_status[ DOOR ] = home_status[ DOOR ];
						previous_status[ GATE ] = home_status[ GATE ];

						//printf("Previous state a inizio blinking: GATE : "); print_state( previous_status[ GATE ] );
						//printf("DOOR: "); print_state( previous_status[ DOOR ] );
						home_status[ DOOR ] = AUTO_OPENING;
						home_status[ GATE ] = AUTO_OPENING;
						out_command = START_AUTO_OPENING;

						sprintf( out_msg, "%d", out_command );
						packetbuf_copyfrom( out_msg, strlen( out_msg)+1 );

						broadcast_send( &broadcast );
					}
					show_available_commands();
					break;

				case INTERNAL_TEMP_C: //Temperature mean command.  It is possible to issue it only if the alarm is deactivated,
					//Does not matter save the state in this case
					if( home_status[ ALARM ] == ON ) {
						printf("[central unit]: ALARM ACTIVE! COMMAND NOT VALID: DEACTIVATE ALARM BEFORE\n");
					} else {
						// It is possible to issue the command
						out_command = INTERNAL_TEMP;
						sprintf( out_msg, "%d", out_command );
						r_send( out_msg, strlen( out_msg) + 1, DOOR_NODE_ADDRESS_0, DOOR_NODE_ADDRESS_1 );
					}
					show_available_commands();
					break;

				case GARDEN_LIGHTS_C: //External light command. It is possible to issue it only if the alarm is deactivated,
					//Does not matter save the state in this case
					if( home_status[ ALARM ] == ON ) {
						printf("[central unit]: ALARM ACTIVE! COMMAND NOT VALID: DEACTIVATE ALARM BEFORE\n");
					} else {
						// It is possible to issue the command
						out_command = EXTERNAL_LIGHT;
						sprintf( out_msg, "%d", out_command );
						r_send( out_msg, strlen( out_msg) + 1, GATE_NODE_ADDRESS_0, GATE_NODE_ADDRESS_1 );
					}
					show_available_commands();
					break;

				case GARAGE_C: //Garage command. It is possible to issue it only if the alarm is deactivated
					if( (home_status[ ALARM ] == ON) || ( home_status[ GARAGE ] == MOVING ) ) {
						if( home_status[ ALARM ] == ON ) printf("[central unit]: ALARM ACTIVE! COMMAND NOT VALID: DEACTIVATE ALARM BEFORE\n");
						else printf( "[central unit]: GARAGE IS MOVING! PLEASE WAITING FOR END OF THE MOVING\n" );
					} else {//It is possible to issue the command

						previous_status[ GARAGE ] = home_status[ GARAGE ];

						if( home_status[ GARAGE ] == CLOSED )
							out_command = GARAGE_OPEN;
						else if( home_status[ GARAGE ] == CLOSED ) //GARAGE IS OPEN
							out_command = GARAGE_CLOSED;

						home_status[ GARAGE ] = MOVING;

						sprintf( out_msg, "%d", out_command );
						r_send( out_msg, strlen( out_msg) + 1, GARAGE_NODE_ADDRESS_0, GARAGE_NODE_ADDRESS_1 );
					}
					show_available_commands();
					break;

				default:
					printf("[central unit]: COMMAND NOT VALID\n");
					break;
			}

		} else if ( ev == sensor_message_event ) { // A message from a sensor node has been received

			strcpy( in_msg, (char*)data );//Message format:	command_code-[value]     value not always is needed

			command = strtok( in_msg, "-");
			value = strtok( NULL, "-");
			i_command = atoi( command );//Convert in integer
			//printf( "Sensor message event: %s, that is Command: ", (char*)data); print_command( i_command );
			//printf("Value: %s\n", value);

			if( (last_sender[0] == GATE_NODE_ADDRESS_0) && (last_sender[1] == GATE_NODE_ADDRESS_1) ) {
				if( i_command == END_AUTO_OPENING ) {// Auto opening stop message.
					//printf("[central unit]: Previous status fine blinking: GATE : "); print_state( previous_status[ GATE ] );
					home_status[ GATE ] = previous_status[ GATE ];//Restoring

					if( inserted_alarm_during_auto_opening == true ) //reset the variable
						inserted_alarm_during_auto_opening = false;
					//printf("[central unit]: Restore status fine blinking: GATE : "); print_state( home_status[ GATE ] );
				}
				else if( i_command == EXTERNAL_LIGHT )// GARDEN_LIGHT
					printf("[central unit]: garden external light value is %s\n", value );
			}
			else if( (last_sender[0] == DOOR_NODE_ADDRESS_0) && (last_sender[1] == DOOR_NODE_ADDRESS_1) ) {
				if( i_command == END_AUTO_OPENING ) {// Auto opening stop message.
					//printf("[central unit]: Previous status fine blinking: DOOR : "); print_state( previous_status[ DOOR ] );
					home_status[ DOOR ] = previous_status[ DOOR ];//Restoring
					if( inserted_alarm_during_auto_opening == true ) //reset the variable
						inserted_alarm_during_auto_opening = false;
					//printf("[central unit]: Restore status fine blinking: DOOR : "); print_state( home_status[ DOOR ] );
				}
				else //INTERNAL TEMP
					printf("[central unit]: internal temperature mean value is %s\n", value );
			}
			else if( (last_sender[0] == GARAGE_NODE_ADDRESS_0) && (last_sender[1] == GARAGE_NODE_ADDRESS_1) ) {

				temp_status = i_command;//Because garage responds just with a command that represent the status
				if( temp_status == OPEN ||  temp_status == CLOSED  || temp_status == MOVING ) {
					//printf("[central unit]: garage status: %s\n", temp_status == CLOSED? "CLOSED" : temp_status == OPEN? "OPEN" : "MOVING");
					if( home_status[ ALARM ] == ON )
						previous_status[ GARAGE ] = temp_status;//Modify the previous and not the home_status because the previous will be restore
					else //ALARM OFF
						home_status[ GARAGE ] = temp_status;//Modify directly the home_status because will NOT be restore
				}

			}
			show_available_commands();

		}

	}//end while
	PROCESS_END();
	return 0;
}


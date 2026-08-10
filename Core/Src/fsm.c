/*
 * fsm.c
 *
 *  Created on: Aug 10, 2026
 *      Author: k.rodolfo
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "drv8323.h"

 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every communication cycle */

	 /* state transition management */
	 if(fsmstate->next_state != fsmstate->state){
		 fsm_exit_state(fsmstate);		// safely exit the old state
		 if(fsmstate->ready){			// if the previous state is ready, enter the new state
			 fsmstate->state = fsmstate->next_state;
			 fsm_enter_state(fsmstate);
		 }
	 }

	 // This is where we do the work required for each state.
	 switch(fsmstate->state){
		 case COMMAND_MODE:
			 break;

		 case TRANSPARENCY_MODE:
			 break;

		 case CONFIG_MODE:
			 break;
	 }

 }

 void fsm_enter_state(FSMStruct * fsmstate){
	 /* Called when entering a new state
	  * Do necessary setup   */

		switch(fsmstate->state){
				case COMMAND_MODE:
				//printf("Entering Command Mode\r\n");
				enter_command_state();
				break;
			case TRANSPARENCY_MODE:
				//printf("Entering Transparency Mode\r\n");
				enter_transparency_state();
				break;
			case CONFIG_MODE:
				//printf("Entering Configuration Mode\r\n");
				enter_config_state();
				break;

		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
			case COMMAND_MODE:
				//printf("Leaving Command Mode\r\n");
				fsmstate->ready = 1;
				break;
			case TRANSPARENCY_MODE:
				//printf("Leaving Transparency Mode\r\n");
				fsmstate->ready = 1;
				break;
			case CONFIG_MODE:
				//printf("Leaving Configuration Mode\r\n");
				fsmstate->ready = 1;
				break;
		}

 }

 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * via serial terminal input or button input
	  */
	if(fsm_input == COMMAND_CMD){
		fsmstate->next_state = COMMAND_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
		case COMMAND_MODE:
			if(fsm_input == TRANSPARENCY_TGL){
				fsmstate->next_state = TRANSPARENCY_MODE;
				fsmstate->ready = 0;
				break;
			}
			break;
		case TRANSPARENCY_MODE:
			if(fsm_input == TRANSPARENCY_TGL){
				fsmstate->next_state = COMMAND_MODE;
				fsmstate->ready = 0;
				break;
			}
			break;
		case CONFIG_MODE:
			break;
	}
	//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
 }


 void enter_menu_state(void){
	    //drv.disable_gd();
	    //reset_foc(&controller);
	    //gpio.enable->write(0);
	    printf("\n\r\n\r");
	    printf(" Commands:\n\r");
	    printf(" m - Motor Mode\n\r");
	    printf(" c - Calibrate Encoder\n\r");
	    printf(" s - Setup\n\r");
	    printf(" e - Display Encoder\n\r");
	    printf(" z - Set Zero Position\n\r");
	    printf(" esc - Exit to Menu\n\r");

	    //gpio.led->write(0);
 }



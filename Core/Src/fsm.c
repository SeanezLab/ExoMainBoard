/*
 * fsm.c
 *
 *  Created on: Aug 10, 2026
 *      Author: k.rodolfo
 */

#include "fsm.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "structs.h"
#include "test_signals.h"
#include "data_tx_arrays.h"
#include "crc.h"

static void run_motor_loop(void);
static void run_com_loop(void);
void run_transparency_loop(void);

 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every iteration of the main loop */

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
			  if (com_loop_flag == 1)
			  {
				  run_com_loop();
			  }
			  if (m_cmd_loop_flag == 1)
			  {
				  run_motor_loop();
			  }
			 break;

		 case TRANSPARENCY_MODE:
			 // Behaves identically to command mode, but with zero gains
			  if (com_loop_flag == 1)
			  {
				  run_com_loop();
			  }
			  if (m_cmd_loop_flag == 1)
			  {
				  run_transparency_loop();
			  }
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
			// Right now you shouldn't be in the state, so just send the user to transparency mode
			fsmstate->next_state = TRANSPARENCY_MODE;
			fsmstate->ready = 0;
			break;
	}
	//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
 }


 void enter_config_state(void)
 {
	// For now, the config state has not been implemented, do nothing.
	 ;
 }

 void enter_command_state(void)
 {

	 // Clear out any queued trajectories
	reset_target_pos(&m1_traj, &m1_cmd);
	reset_target_pos(&m2_traj, &m2_cmd);
	 // Zero out any of the commands

	 // Re-apply the gains
	 reapply_motor_gains(&m1_cmd);
	 reapply_motor_gains(&m2_cmd);
 }
 void enter_transparency_state(void)
 {
	 // Save the current control gains and zero out motor 1
	 save_motor_gains(&m1_cmd);
	 zero_motor_gains(&m1_cmd);
	 m1_cmd.new_cont = 1;
	 // Do the same for motor 2
	 save_motor_gains(&m2_cmd);
	 zero_motor_gains(&m2_cmd);
	 m2_cmd.new_cont = 1;
	 // Clear the trajectories for both the motors (TODO)
	 // Immediately update the motors
	handle_m_cmd(&m1_cmd, &m1_tx);
	handle_m_cmd(&m2_cmd, &m2_tx);

 }


void run_motor_loop(void)
{
	/* This is the loop that generates trajectory commands and sends them to
	 * the motors.
	 */
	// Check for new commands from bluetooth if this pathway is active
	if (got_bt_msg == true && UART_PORT == 1)
	{
	  dma_to_rdg_buf(bt_dma_reader, bt_rx_dma_buffer, bt_msg_size);
	  crc_uart_rcv_data(bt_dma_reader, bt_msg_size);
	  flush_buffer(bt_dma_reader);
	  got_bt_msg = false;
	}
	// Same for USART2
	if (got_usart2_msg == true && UART_PORT == 2)
		{
		  dma_to_rdg_buf(usart2_dma_reader, usart2_dma_buffer, usart2_msg_size);
		  crc_uart_rcv_data(usart2_dma_reader, usart2_msg_size);
		  flush_buffer(usart2_dma_reader);
		  got_usart2_msg = false;
	}
	// Update trajectory
	advance_traj(&m1_traj, &m1_cmd);
	advance_traj(&m2_traj, &m2_cmd);
	// Handle Commands
	// Query idle motors at the acquisition rate, independent of UART draining.
	m1_cmd.new_query = 1;
	m2_cmd.new_query = 1;
	handle_m_cmd(&m1_cmd, &m1_tx);
	handle_m_cmd(&m2_cmd, &m2_tx);
	// Turn off flag
	m_cmd_loop_flag = 0;
}

void run_transparency_loop(void)
{
	// Check for new commands from bluetooth if this pathway is active
	if (got_bt_msg == true && UART_PORT == 1)
	{
	  dma_to_rdg_buf(bt_dma_reader, bt_rx_dma_buffer, bt_msg_size);
	  crc_uart_rcv_data(bt_dma_reader, bt_msg_size);
	  flush_buffer(bt_dma_reader);
	  got_bt_msg = false;
	}
	// Same for USART2
	if (got_usart2_msg == true && UART_PORT == 2)
		{
		  dma_to_rdg_buf(usart2_dma_reader, usart2_dma_buffer, usart2_msg_size);
		  crc_uart_rcv_data(usart2_dma_reader, usart2_msg_size);
		  flush_buffer(usart2_dma_reader);
		  got_usart2_msg = false;
	}
	// Enforce Transparency (We don't set the new command flag, just ensure that gains are zero)
//	 save_motor_gains(&m1_cmd);
	 zero_motor_gains(&m1_cmd);
//	 save_motor_gains(&m2_cmd);
	 zero_motor_gains(&m2_cmd);

	// Handle Commands
	m1_cmd.new_query = 1;
	m2_cmd.new_query = 1;
	handle_m_cmd(&m1_cmd, &m1_tx);
	handle_m_cmd(&m2_cmd, &m2_tx);
	// Turn off flag
	m_cmd_loop_flag = 0;
}

void run_com_loop(void)
{
	// Submit queued snapshots as ordinary back-to-back packets. Never wait for UART.
	data_tx_history_drain();
	com_loop_flag = 0;
}


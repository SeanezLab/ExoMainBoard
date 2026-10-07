/*
 * cmd_array.c
 *
 * The purpose of this file is to hold all the commands sent to the exoskeleton main board. During the main control loop, commands will be
 * taken from this array and executed. This is not a queue. If the state of the exoskeleton matches the command arrays, nothing should change.
 * I will adapt this array if this does not hold.
 *
 *  Created on: Jan 28, 2026
 *      Author: k.rodolfo
 */


#include "cmd_array.h"

// Exoskeleton desired state/mode
float des_mode = 0;

void motor_cmd_init(MotorCommand* m_cmd, uint8_t motor_id)
{
	m_cmd->motor_id = motor_id;
	m_cmd->des_pos = 0; //Initialize with all zeros
	m_cmd->des_mode = 0; //Start with the motor disabled
	m_cmd->last_mode = 0; // Set the last mode to 0.
	m_cmd->des_v = 0;
	m_cmd->svd_v = 0;
	if (motor_id == 1)
	{
		m_cmd->des_kp = DES_M1_KP;
		m_cmd->svd_kp = DES_M1_KP;
		m_cmd->des_kd = DES_M1_KD;
		m_cmd->svd_kd = DES_M1_KD;
	}
	else if (motor_id == 2)
	{
		m_cmd->des_kp = DES_M2_KP;
		m_cmd->svd_kp = DES_M2_KP;
		m_cmd->des_kd = DES_M2_KD;
		m_cmd->svd_kd = DES_M2_KD;
	}
	else
	{
		m_cmd->des_kp = DEF_KP;
		m_cmd->svd_kp = DEF_KP;
		m_cmd->des_kd = DEF_KD;
		m_cmd->svd_kd = DEF_KD;
	}
	m_cmd->des_tff = 0;
	m_cmd->svd_tff = 0;
	m_cmd->new_pos = 0; //Start with the new position flag off
	m_cmd->new_sp_cmd = 1; //Start with the new command on so we can set the motor to disable on startup
	m_cmd->new_cont = 0;
	m_cmd->new_query = 0;
	m_cmd->command_mode = CAN_COMMAND_ENCODER; // CAN_COMMAND
	m_cmd->query_mode = CAN_QUERY_ENCODER;
}

void vibro_cmd_init(VibroCommand* vibro_cmd)
{
	vibro_cmd->des_duty_cycle = 0;
	vibro_cmd->des_targ_freq = 0;
	vibro_cmd->des_tscs_delay = 0;
	vibro_cmd->des_vibro_zthresh = 0;
	vibro_cmd->new_delay = 0;
	vibro_cmd->new_duty_cycle = 0;
	vibro_cmd->new_freq = 0;
	vibro_cmd->new_zthresh = 0;
}


static bool send_m_can(CANTxMessage* m_tx)
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &(m_tx->tx_header), m_tx->data) != HAL_OK)
	{
		HAL_GPIO_WritePin(Debug_GPIO_Port, Debug_Pin, GPIO_PIN_SET);
		return false;
	}
	return true;
}

void handle_m_cmd(MotorCommand* m_cmd, CANTxMessage* m_tx)
{
	// Special commands take precedence
	if (m_cmd->new_sp_cmd == 1)
	{
		CANSpecialCommand command;
		if (m_cmd->des_mode == 0){command = CAN_SPECIAL_DISABLE;}
		else if (m_cmd->des_mode == 1){command = CAN_SPECIAL_ENABLE;}
		else if (m_cmd->des_mode == 2){command = CAN_SPECIAL_ZERO;}
		else
		{
			return;
		}

		if (!can_pack_special(m_tx, command) || !send_m_can(m_tx))
		{
			return;
		}
		m_cmd->new_sp_cmd = 0;

	}
	// A new position or a new control scheme is given
	else if (m_cmd->new_pos == 1 || m_cmd->new_cont == 1)
	{
		CANCommandData command = {
			.p_des = m_cmd->des_pos,
			.v_des = m_cmd->des_v,
			.kp = m_cmd->des_kp,
			.kd = m_cmd->des_kd,
			.t_ff = m_cmd->des_tff
		};
		if (!can_pack_tx(m_tx, &command, m_cmd->command_mode) || !send_m_can(m_tx))
		{
			return; // return on failure
		}
		m_cmd->new_pos = 0;
		m_cmd->new_cont = 0;
	}

	// Pending queries use otherwise idle calls
	else if (m_cmd->new_query == 1)
	{
		if (can_pack_query(m_tx, m_cmd->query_mode) && send_m_can(m_tx))
		{
			m_cmd->new_query = 0;
		}
	}
}

void reapply_motor_gains(MotorCommand* m_cmd)
{
	// Reapplies the saved motor gains
	 m_cmd->des_kd = m_cmd->svd_kd;
	 m_cmd->des_kp = m_cmd->svd_kp;
	 m_cmd->des_tff = m_cmd->svd_tff;
	 m_cmd->des_v = m_cmd->svd_v;
}
void save_motor_gains(MotorCommand* m_cmd)
{
	 /* Saves the currently set motor gains
	  */
	 m_cmd->svd_kd = m_cmd->des_kd;
	 m_cmd->svd_kp = m_cmd->des_kp;
	 m_cmd->svd_tff = m_cmd->des_tff;
	 m_cmd->svd_v = m_cmd->des_v;
}

void zero_motor_gains(MotorCommand* m_cmd)
{
	/* Zeros our the motor gains
	 */
	 m_cmd->des_kd = DEF_KP;
	 m_cmd->des_kp = DEF_KD;
	 m_cmd->des_tff = DEF_TFF;
	 m_cmd->des_v = DEF_V;
}

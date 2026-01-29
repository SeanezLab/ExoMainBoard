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
	m_cmd->des_v = 0;
	m_cmd->des_kp = 1;
	m_cmd->des_kd = 0;
	m_cmd->des_tff = 0;
	m_cmd->new_pos = 0; //Start with the new position flag off
	m_cmd->new_sp_cmd = 1; //Start with the new command on so we can set the motor to disable on startup
	m_cmd->rdy_to_snd = 1;
	m_cmd->new_cont = 0;
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

void handle_m_cmd(MotorCommand* m_cmd, CANTxMessage* m_tx)
{
	if (m_cmd->new_pos == 1)
	{
		// Might have to change this to be instance specific if the motors have different control weights
		can_pack_tx(m_tx, &(m_cmd->des_pos), &(m_cmd->des_v), &(m_cmd->des_kp), &(m_cmd->des_kd), &(m_cmd->des_tff));
		m_cmd->new_pos = 0;
		m_cmd->rdy_to_snd = 1;
	}

	if (m_cmd->new_sp_cmd == 1)
	{
		// Exit Motor Mode
		if (m_cmd->des_mode == 0)
		{
			m_tx->data[0] = 0xff;
			m_tx->data[1] = 0xff;
			m_tx->data[2] = 0xff;
			m_tx->data[3] = 0xff;
			m_tx->data[4] = 0xff;
			m_tx->data[5] = 0xff;
			m_tx->data[6] = 0xff;
			m_tx->data[7] = 0xfd;//fd
			m_cmd->new_sp_cmd = 0;
			m_cmd->rdy_to_snd = 1;
		}
		// Enter Motor Mode
		else if (m_cmd->des_mode == 1)
		{
			m_tx->data[0] = 0xff;
			m_tx->data[1] = 0xff;
			m_tx->data[2] = 0xff;
			m_tx->data[3] = 0xff;
			m_tx->data[4] = 0xff;
			m_tx->data[5] = 0xff;
			m_tx->data[6] = 0xff;
			m_tx->data[7] = 0xfc;
			m_cmd->new_sp_cmd = 0;
			m_cmd->rdy_to_snd = 1;
		}
		// Zero Position Sensor
		else if (m_cmd->des_mode == 2)
		{
			m_tx->data[0] = 0xff;
			m_tx->data[1] = 0xff;
			m_tx->data[2] = 0xff;
			m_tx->data[3] = 0xff;
			m_tx->data[4] = 0xff;
			m_tx->data[5] = 0xff;
			m_tx->data[6] = 0xff;
			m_tx->data[7] = 0xfe;//fe
			m_cmd->new_sp_cmd = 0;
			m_cmd->rdy_to_snd = 1;
		}
		else
		{
			return; // Could not parse special command
		}
	}
	if (m_cmd->new_cont == 1)
		{
			// Might have to change this to be instance specific if the motors have different control weights
			can_pack_tx(m_tx, &(m_cmd->des_pos), &(m_cmd->des_v), &(m_cmd->des_kp), &(m_cmd->des_kd), &(m_cmd->des_tff));
			m_cmd->new_cont = 0;
			m_cmd->rdy_to_snd = 1;
		}
}

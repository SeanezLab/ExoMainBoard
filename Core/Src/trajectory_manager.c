/*
 * trajectory_manager.c
 *
 *  Created on: Mar 4, 2026
 *      Author: k.rodolfo
 */

#include "trajectory_manager.h"
#include "test_signals.h"
#include "tim.h"
#include "cmd_array.h"
#include "data_tx_arrays.h"
#include <string.h>
#include <math.h>

void motor_trajectory_init(MotorTrajectory* m_traj, uint8_t motor_id)
{
	m_traj->motor_id = motor_id;
	m_traj->traj_mode = 0; //0 is idle
	m_traj->cmd_idx = 0;
	memset(m_traj->pos_array, 0, sizeof(m_traj->pos_array));
	m_traj->t_mult = 5;
	m_traj->des_freq = 1;
	m_traj->theta_target = 0;
	m_traj->time_to_targ = 10; // In sec
	m_traj->joint_inertia_ff;
	m_traj->gravity_ff = 0;
	m_traj->dyn_frct_ff = 0;
	m_traj->stat_frct_ff = 1.5;
	m_traj->trans_v_ff = 0.1;
	m_traj->theta = 0;
	m_traj->theta_d = 0;
	m_traj->theta_dd = 0;
	m_traj->tic = 0;
	m_traj->cmd_rdy = 0;
	m_traj->new_traj_req = 0;
	m_traj->traj_cmplt = 0;
}

void advance_traj(MotorTrajectory* m_traj, MotorCommand* m_cmd)
{

	m_traj->tic += 1;

	if (m_traj->tic == m_traj->t_mult)
	{
		generate_traj_cmd(m_traj, m_cmd);
		m_traj->tic = 0;
	}

}

void generate_traj_cmd(MotorTrajectory* m_traj, MotorCommand* m_cmd)
{
	switch(m_traj->traj_mode)
	{
	case 0:
//		if (m_traj->new_traj_req == true)
//		{
//			m_traj->new_traj_req = false;
//			m_cmd->des_tff = 0;
//			m_cmd->new_cont = 1;
//			return;
//		}
		return;

	case 1:// Sinusoid
		m_traj->theta = traj_cos_theta(m_traj->des_freq, traj_clock);
		m_traj->theta_d = traj_cos_theta_dot(m_traj->des_freq, traj_clock);
		m_cmd->des_pos = m_traj->theta;
		m_cmd->des_v = m_traj->theta_d;
		m_cmd->new_pos = 1;
		m_cmd->new_cont = 1;
		return;
	case 2:
		if (m_traj->new_traj_req == true)
		{
			m_traj->new_traj_req = false;
			float dt = 0.005f * m_traj->t_mult;
			minjerk_start(&(m_traj->jerk_traj), m_cmd->des_pos, m_traj->theta_target, m_traj->time_to_targ, dt);
			minjerk_step(&(m_traj->jerk_traj), &(m_traj->theta), &(m_traj->theta_d), &(m_traj->theta_dd));
		}
		else if (m_traj->jerk_traj.active == true)
		{
			minjerk_step(&(m_traj->jerk_traj), &(m_traj->theta), &(m_traj->theta_d), &(m_traj->theta_dd));
		}
		memcpy(m1_des, &(m_traj->theta), sizeof(float));
		float t_ff = friction_ff(m_traj->theta_d, m_traj->dyn_frct_ff, m_traj->stat_frct_ff, m_traj->trans_v_ff);
		m_cmd->des_pos = m_traj->theta;
		m_cmd->des_v = m_traj->theta_d;
		m_cmd->des_tff = t_ff;
		m_cmd->new_pos = 1;
		m_cmd->new_cont = 1;

	}
}

void minjerk_start(MinJerkTraj* tr, float theta0, float thetaf, float T, float dt)
{
	tr->theta0 = theta0;
	tr->thetaf = thetaf;
	tr->T = (T > 1e-6F) ? T : 1e-6F;
	tr->t = 0.0F;
	tr->dt = dt;
	tr->active = true;
}


// Returns false when the trajectory is finished.
bool minjerk_step(MinJerkTraj* tr, float* theta, float* theta_dot, float* theta_ddot)
{
	if (tr->active == false)
	{
		// Trajectory finished, hold the final
		if (theta) *theta = tr->thetaf;
		if (theta_dot) *theta_dot = 0.0f;
		if (theta_ddot) *theta_ddot = 0.0f;
	}

	// Clamp time to [0, T]
	float t = tr->t;
	if (t >= tr->T) t = tr->T;

	const float T = tr->T;
	const float s = t/T; // normalized time [0, 1]
	const float s2 = s * s;
	const float s3 = s2 * s;
	const float s4 = s3 * s;
	const float s5 = s4 * s;

	const float dtheta = tr->thetaf - tr->theta0;

	// p(s) = 10 s^3 - 15 s^4 + 6 s^5
	const float p   = 10.0f*s3 - 15.0f*s4 +  6.0f*s5;

	// p'(s) = 30 s^2 - 60 s^3 + 30 s^4
	const float dp  = 30.0f*s2 - 60.0f*s3 + 30.0f*s4;

	// p''(s)= 60 s - 180 s^2 + 120 s^3
	const float ddp = 60.0f*s  - 180.0f*s2 + 120.0f*s3;

    if (theta)      *theta      = tr->theta0 + dtheta * p;
    if (theta_dot)  *theta_dot  = (dtheta / T)  * dp;
    if (theta_ddot) *theta_ddot = (dtheta / (T*T)) * ddp;

    // advance time
    tr->t += tr->dt;

    // finish?
    if (tr->t >= tr->T + 0.5f*tr->dt) {
        tr->active = false;
        // Optionally hard-set final for cleanliness
        if (theta)      *theta      = tr->thetaf;
        if (theta_dot)  *theta_dot  = 0.0f;
        if (theta_ddot) *theta_ddot = 0.0f;
        return false;
    }

    return true;
}

// Smooth sign function using tanh(v/v0)
// v0 sets how quickly it transitions near 0 (units: position_units/s)
float smooth_sign(float v, float v0)
{
    // avoid divide-by-zero
    if (v0 < 1e-6f) v0 = 1e-6f;
    return tanhf(v / v0);   // in [-1, 1]
}

// Friction feedforward torque model:
// tau_ff = b * v_des + tau_c * tanh(v_des / v0)
float friction_ff(float v_des, float b_visc, float tau_breakaway, float v0)
{
    return b_visc * v_des + tau_breakaway * smooth_sign(v_des, v0);
}

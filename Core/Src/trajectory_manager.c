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
#include "structs.h"
#include <string.h>
#include <math.h>

#define DT 0.00033333f

float t_ff;
static void abort_logchirp(MotorTrajectory* m_traj, MotorCommand* m_cmd, ChirpAbortReason reason);

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
	m_traj->chirp_start_freq = 0.5f;
	m_traj->chirp_end_freq = 50.0f;
	m_traj->chirp_duration = 60.0f;
	m_traj->chirp_amplitude = 0.0f; // No excitation until explicitly commanded
	m_traj->chirp_ramp_time = 1.0f;
	m_traj->chirp_bias = 0.0f;
	memset(&m_traj->chirp_traj, 0, sizeof(m_traj->chirp_traj));
}

void advance_traj(MotorTrajectory* m_traj, MotorCommand* m_cmd)
{

	m_traj->tic += 1;

	// Check if we are going from disable to enabled motors. If so, set the desired position to the current position.
	if ((m_cmd->new_sp_cmd == 1) && (m_cmd->last_mode != 1) && (m_cmd->des_mode == 1))
	{
		reset_target_pos(m_traj,m_cmd);
	}

	// Update the timer
	// Chirps and their abort checks run every motor-loop service, independently
	// of the decimation used by the position trajectories.
	if (m_traj->traj_mode == TRAJ_LOG_CHIRP || m_traj->tic == m_traj->t_mult)
	{
		generate_traj_cmd(m_traj, m_cmd);
		m_traj->tic = 0;
	}

	// Update the trajectory mode tx array
	if (m_traj->motor_id == 1)
	{
		memcpy(m1_traj_status, &(m_traj->traj_cmplt), sizeof(bool));
	}

	if (m_traj->motor_id == 2)
	{
		memcpy(m2_traj_status, &(m_traj->traj_cmplt), sizeof(bool));
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

	case 2:// Minimum Jerk
		if (m_traj->new_traj_req == true)
		{
			m_traj->new_traj_req = false;
			float dt = 0.0002f * m_traj->t_mult;
			minjerk_start(&(m_traj->jerk_traj), m_cmd->des_pos, m_traj->theta_target, m_traj->time_to_targ, dt);
			m_traj->traj_cmplt = m_traj->jerk_traj.active;
		}
		else if (m_traj->jerk_traj.active == true)
		{
			minjerk_step(&(m_traj->jerk_traj), &(m_traj->theta), &(m_traj->theta_d), &(m_traj->theta_dd));
			m_traj->traj_cmplt = m_traj->jerk_traj.active;
		}
		else if (m_traj->jerk_traj.active == false)
		{
			m_traj->traj_cmplt = m_traj->jerk_traj.active;
		}

		t_ff = friction_ff(m_traj->theta_d, m_traj->dyn_frct_ff, m_traj->stat_frct_ff, m_traj->trans_v_ff);
		m_cmd->des_pos = m_traj->theta;
		m_cmd->des_v = m_traj->theta_d;
		m_cmd->des_tff = t_ff;
		m_cmd->new_pos = 1;
		m_cmd->new_cont = 1;
		return;

	case 3:// Constant Velocity
		if (m_traj->new_traj_req == true)
		{
			m_traj->new_traj_req = false;
			float dt = 0.0002f * m_traj->t_mult;
			constvel_start(&(m_traj->const_vel_traj), m_cmd->des_pos, m_traj->theta_target, m_traj->time_to_targ, dt);
			m_traj->traj_cmplt = m_traj->jerk_traj.active;
		}
		else if (m_traj->const_vel_traj.active == true)
		{
			constvel_step(&(m_traj->const_vel_traj), &(m_traj->theta), &(m_traj->theta_d), &(m_traj->theta_dd));
			m_traj->traj_cmplt = m_traj->const_vel_traj.active;
		}
		else if (m_traj->const_vel_traj.active == false)
		{
			m_traj->traj_cmplt = m_traj->const_vel_traj.active;
		}

		t_ff = friction_ff(m_traj->theta_d, m_traj->dyn_frct_ff, m_traj->stat_frct_ff, m_traj->trans_v_ff);
		m_cmd->des_pos = m_traj->theta;
		m_cmd->des_v = m_traj->theta_d;
		m_cmd->des_tff = t_ff;
		m_cmd->new_pos = 1;
		m_cmd->new_cont = 1;
		return;

	case TRAJ_LOG_CHIRP: // Torque disturbance around a fixed position reference
	{
		LogChirpTraj* tr = &m_traj->chirp_traj;
		if (m_cmd->des_mode != 1 || m_cmd->new_sp_cmd)
		{
			cancel_logchirp(m_traj, m_cmd);
			return;
		}
		if (m_traj->new_traj_req == true)
		{
			m_traj->new_traj_req = false;
			if (!isfinite(m_cmd->des_pos) ||
				!logchirp_start(tr, m_traj->chirp_start_freq, m_traj->chirp_end_freq,
					m_traj->chirp_duration, m_traj->chirp_amplitude,
					m_traj->chirp_ramp_time, m_traj->chirp_bias, motor_loop_period_seconds()))
			{
				abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_PARAMETERS);
				return;
			}
			tr->hold_position = m_cmd->des_pos;
			tr->start_tick = motor_loop_ticks;
			m_traj->jerk_traj.active = false;
			m_traj->const_vel_traj.active = false;
			m_traj->tic = 0;
			m_cmd->command_mode = CAN_COMMAND_CHARACTERIZATION;
			m_cmd->query_mode = CAN_QUERY_CHARACTERIZATION;
		}

		// Snapshot the latest driver states together. They are in driver coordinates,
		// unlike m1_pos, whose sign is reversed for host telemetry.
		volatile CANMotorTelemetry* telemetry;
		if (m_traj->motor_id == 1)
		{
			telemetry = &m1_can_telemetry;
		}
		else if (m_traj->motor_id == 2)
		{
			telemetry = &m2_can_telemetry;
		}
		else
		{
			abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_PARAMETERS);
			return;
		}
		uint32_t irq_state = __get_PRIMASK();
		__disable_irq();
		CANMotorTelemetry latest = *telemetry;
		__set_PRIMASK(irq_state);
		CANCharacterizationReply* sample = &latest.characterization;
		if (latest.characterization_count == 0 || latest.last_reply_mode != CAN_REPLY_CHARACTERIZATION ||
			(uint32_t)(HAL_GetTick() - latest.last_reply_ms) >= CHIRP_FEEDBACK_TIMEOUT_MS ||
			!isfinite(sample->position) || !isfinite(sample->i_q) || !isfinite(sample->i_q_des))
		{
			abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_FEEDBACK);
			return;
		}
//		if (fabsf(sample->position - tr->hold_position) >= CHIRP_POSITION_LIMIT_RAD ||
//			sample->position <= -32768.0f * CAN_CHARACTERIZATION_P_STEP ||
//			sample->position >= 32767.0f * CAN_CHARACTERIZATION_P_STEP)
//		{
//			abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_POSITION);
//			// Saturated +/-pi telemetry cannot support a reliable excursion check.
//			return;
//		}
		// Check both measured and total commanded current, not just the chirp amplitude.
		if (fabsf(sample->i_q) * KT * (float)GR >= CHIRP_TORQUE_LIMIT_NM ||
			fabsf(sample->i_q_des) * KT * (float)GR >= CHIRP_TORQUE_LIMIT_NM)
		{
			abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_TORQUE);
			return;
		}

		uint32_t elapsed_ticks = motor_loop_ticks - tr->start_tick;
		logchirp_step(tr, elapsed_ticks, &m_cmd->des_tff);
		if (tr->abort_reason != CHIRP_ABORT_NONE || !isfinite(m_cmd->des_tff))
		{
			abort_logchirp(m_traj, m_cmd, CHIRP_ABORT_PARAMETERS);
			return;
		}
		m_traj->theta = tr->hold_position;
		m_traj->theta_d = 0.0f;
		m_traj->theta_dd = 0.0f;
		m_traj->traj_cmplt = tr->active; // Existing telemetry convention: true while active
		m_cmd->des_pos = tr->hold_position;
		m_cmd->des_v = 0.0f;
		m_cmd->new_pos = 1;
		m_cmd->new_cont = 1;
		float position_tx = (m_traj->motor_id == 1) ? -tr->hold_position : tr->hold_position;

	}
	}
}

void reset_target_pos(MotorTrajectory* m_traj, MotorCommand* m_cmd)
{
	// Re-enabling or changing FSM mode must not restart an old sweep.
	cancel_logchirp(m_traj, m_cmd);
	// Sets the target position of the motor to the current position. Used when transitioning
	// from a disabled->enabled motor state, or from the transparent->command state for the exo
	m_traj->new_traj_req = true;
	// Update the desired position from the latest read position
	float latest_pos;
	if (m_traj->motor_id == 1)
	{

		memcpy(&latest_pos, m1_pos, sizeof(latest_pos));
		m_cmd->des_pos = latest_pos * -1; //Flip sign for the m1 motor.
		m_traj->theta_target =  latest_pos * -1;
	}

	if (m_traj->motor_id == 2)
	{
		memcpy(&latest_pos, m2_pos, sizeof(latest_pos));
		m_cmd->des_pos = latest_pos;
		m_traj->theta_target =  latest_pos;
	}
}

bool logchirp_parameters_valid(float start_freq, float end_freq, float duration,
	float amplitude, float ramp_time, float bias, float dt)
{
	// Validate parameters without creating or changing any trajectory state.
	if (!isfinite(start_freq) || !isfinite(end_freq) || !isfinite(duration) ||
		!isfinite(amplitude) || !isfinite(ramp_time) || !isfinite(bias) || !isfinite(dt) ||
		dt <= 0.0f || start_freq <= 0.0f || end_freq <= 0.0f || duration < dt ||
		amplitude < 0.0f || ramp_time < 0.0f || ramp_time > duration * 0.5f ||
		fabsf(bias) + amplitude >= CHIRP_TORQUE_LIMIT_NM ||
		fmaxf(start_freq, end_freq) * dt >= 0.5f || duration >= (double)UINT32_MAX * dt)
	{
		return false;
	}
	return true;
}

bool logchirp_start(LogChirpTraj* tr, float start_freq, float end_freq, float duration,
	float amplitude, float ramp_time, float bias, float dt)
{
	if (!logchirp_parameters_valid(start_freq, end_freq, duration, amplitude, ramp_time, bias, dt))
	{
		return false;
	}
	memset(tr, 0, sizeof(*tr));
	tr->start_freq = start_freq;
	tr->end_freq = end_freq;
	tr->T = duration;
	tr->dt = dt;
	tr->amplitude = amplitude;
	tr->bias = bias;
	tr->ramp_time = ramp_time;
	tr->log_rate = log((double)end_freq / start_freq) / duration;
	tr->frequency_state = start_freq;
	tr->frequency_multiplier = exp(tr->log_rate * dt);
	const double two_pi = 6.283185307179586;
	tr->phase_per_hz = two_pi * dt;
	if (tr->log_rate != 0.0)
	{
		// Exact integral over one timer period, per Hz at the start of that period.
		tr->phase_per_hz = two_pi * expm1(tr->log_rate * dt) / tr->log_rate;
	}
	tr->frequency = start_freq;
	tr->torque = bias;
	tr->active = true;
	return true;
}

bool logchirp_step(LogChirpTraj* tr, uint32_t elapsed_ticks, float* torque)
{
	if (elapsed_ticks < tr->elapsed_ticks)
	{
		tr->active = false;
		tr->abort_reason = CHIRP_ABORT_PARAMETERS;
	}
	if (tr->abort_reason != CHIRP_ABORT_NONE)
	{
		tr->perturbation = 0.0f;
		tr->torque = 0.0f;
		if (torque){*torque = tr->torque;}
		return false;
	}
	uint32_t ticks_to_advance = elapsed_ticks - tr->elapsed_ticks;
	tr->elapsed_ticks = elapsed_ticks;
	double elapsed_seconds = (double)elapsed_ticks * tr->dt;
	tr->t = fminf((float)elapsed_seconds, tr->T);
	if (!tr->active || elapsed_seconds >= tr->T)
	{
		tr->active = false;
		tr->frequency = tr->end_freq;
		tr->perturbation = 0.0f;
		tr->torque = tr->bias; // Normal completion keeps the holding bias
		if (torque){*torque = tr->torque;}
		return false;
	}

	const double two_pi = 6.283185307179586;
	if (ticks_to_advance == 1)
	{
		// Integrate this interval using its starting frequency, then advance frequency.
		tr->phase += tr->frequency_state * tr->phase_per_hz;
		tr->frequency_state *= tr->frequency_multiplier;
		// Frequencies are below Nyquist, so one tick adds less than pi radians.
		if (tr->phase >= two_pi){tr->phase -= two_pi;}
	}
	else if (ticks_to_advance > 1)
	{
		// A delayed call must not stretch the sweep. Resynchronize directly instead
		// of looping over missed samples. Expensive math is only used on this path.
		if (tr->log_rate == 0.0)
		{
			tr->phase = two_pi * tr->start_freq * elapsed_seconds;
			tr->frequency_state = tr->start_freq;
		}
		else
		{
			tr->phase = two_pi * tr->start_freq * expm1(tr->log_rate * elapsed_seconds) / tr->log_rate;
			tr->frequency_state = tr->start_freq * exp(tr->log_rate * elapsed_seconds);
		}
		tr->phase = fmod(tr->phase, two_pi);
	}
	// Zero elapsed ticks leave the waveform unchanged, including on its first call.
	tr->frequency = (float)tr->frequency_state;
	float envelope = 1.0f;
	float edge_time = fminf(tr->t, tr->T - tr->t);
	if (tr->ramp_time > 0.0f && edge_time < tr->ramp_time)
	{
		envelope = 0.5f - 0.5f * cosf(3.1415926536f * edge_time / tr->ramp_time);
	}
	tr->perturbation = tr->amplitude * envelope * sinf((float)tr->phase);
	tr->torque = tr->bias + tr->perturbation;
	if (torque){*torque = tr->torque;}
	return true;
}

void cancel_logchirp(MotorTrajectory* m_traj, MotorCommand* m_cmd)
{
	if (m_traj->traj_mode != TRAJ_LOG_CHIRP && !m_traj->chirp_traj.active){return;}
	m_traj->chirp_traj.active = false;
	m_traj->chirp_traj.perturbation = 0.0f;
	m_traj->chirp_traj.torque = 0.0f;
	m_traj->traj_mode = 0;
	m_traj->new_traj_req = false;
	m_traj->traj_cmplt = false;
	m_cmd->des_tff = 0.0f;
	m_cmd->des_v = 0.0f;
	m_cmd->new_cont = 1;
}

static void abort_logchirp(MotorTrajectory* m_traj, MotorCommand* m_cmd, ChirpAbortReason reason)
{
	cancel_logchirp(m_traj, m_cmd);
	m_traj->chirp_traj.abort_reason = reason;
	// Disabling takes precedence over control packets and is retried by handle_m_cmd.
	// Explicit host re-enable and a new type-5 command are required to run again.
	m_cmd->last_mode = m_cmd->des_mode;
	m_cmd->des_mode = 0;
	m_cmd->new_sp_cmd = 1;
	m_cmd->new_pos = 0;
	m_cmd->new_cont = 0;
}

void constvel_start(ConstVel* tr, float theta0, float thetaf, float T, float dt)
{
	tr->theta0 = theta0;
	tr->thetaf = thetaf;
	tr->T = (T > 1e-6F) ? T : 1e-6F;
	tr->t = 0.0F;
	tr->dt = dt;
	tr->active = true;
}

bool constvel_step(ConstVel* tr, float* theta, float* theta_dot, float* theta_ddot)
{
    if (tr->active == false)
    {
        // Trajectory finished, hold the final
        if (theta)      *theta      = tr->thetaf;
        if (theta_dot)  *theta_dot  = 0.0f;
        if (theta_ddot) *theta_ddot = 0.0f;
        return false;
    }

    // Protect against divide-by-zero or invalid trajectory time
    if (tr->T <= 0.0f)
    {
        if (theta)      *theta      = tr->thetaf;
        if (theta_dot)  *theta_dot  = 0.0f;
        if (theta_ddot) *theta_ddot = 0.0f;

        tr->active = false;
        return false;
    }

    // Clamp time to [0, T]
    float t = tr->t;
    if (t >= tr->T) t = tr->T;
    if (t <= 0.0f)  t = 0.0f;

    const float T = tr->T;
    const float dtheta = tr->thetaf - tr->theta0;

    // Constant velocity
    const float v = dtheta / T;

    // Linear trajectory:
    // theta(t) = theta0 + v*t
    // theta_dot(t) = v
    // theta_ddot(t) = 0
    if (theta)      *theta      = tr->theta0 + v * t;
    if (theta_dot)  *theta_dot  = v;
    if (theta_ddot) *theta_ddot = 0.0f;

    // Advance time
    tr->t += tr->dt;

    // Finish?
    if (tr->t >= tr->T + 0.5f * tr->dt)
    {
        tr->active = false;

        // Hard-set final for cleanliness
        if (theta)      *theta      = tr->thetaf;
        if (theta_dot)  *theta_dot  = 0.0f;
        if (theta_ddot) *theta_ddot = 0.0f;

        return false;
    }

    return true;
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
		return false;
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

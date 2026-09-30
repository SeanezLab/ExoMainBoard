/*
 * trajectory_manager.h
 *
 *  Created on: Mar 4, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_TRAJECTORY_MANAGER_H_
#define INC_TRAJECTORY_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "main.h"
#include "cmd_array.h"

#define TRAJ_LEN 5000
#define TRAJ_LOG_CHIRP 4U
#define CHIRP_TORQUE_LIMIT_NM 8.0f
#define CHIRP_POSITION_LIMIT_RAD 0.7853981634f // 45 degrees from the held setpoint
#define CHIRP_FEEDBACK_TIMEOUT_MS 100U

typedef struct{
	float theta0;//in radians
	float thetaf;//in radians
	float T; //In Seconds
	float t; //current time (locally), in seconds
	float dt; //timestep, in seconds
	bool active;
} MinJerkTraj;


typedef struct{
	float theta0;//in radians
	float thetaf;//in radians
	float T; //In Seconds
	float t; //current time (locally), in seconds
	float dt; //timestep, in seconds
	bool active;
} ConstVel;

typedef enum{
	CHIRP_ABORT_NONE,
	CHIRP_ABORT_POSITION,
	CHIRP_ABORT_TORQUE,
	CHIRP_ABORT_FEEDBACK,
	CHIRP_ABORT_PARAMETERS
}ChirpAbortReason;

typedef struct{
	float start_freq, end_freq; // Hz
	float T, t, dt; // Seconds; dt is the motor timer period, not COM period
	float amplitude, bias; // Output-side Nm
	float ramp_time, log_rate;
	float frequency, perturbation, torque;
	float hold_position; // Driver coordinates, radians
	uint32_t start_tick;
	bool active;
	ChirpAbortReason abort_reason;
}LogChirpTraj;

typedef struct{
	uint8_t motor_id;
	uint8_t traj_mode; // 0: Free move, 1: Sinusoid, 2: Minimum jerk, 3: Constant velocity, 4: Torque chirp.
	uint32_t cmd_idx;
	float pos_array[TRAJ_LEN];
	uint8_t t_mult; // How many tics of the cmd_loop to wait before generating a new trajectory/ Affects the rate commands are send
	uint8_t des_freq; // How much to scale the input frequency
	float theta_target;
	float theta_current;
	float theta_d_measured;
	float time_to_targ; // In sec;
	float joint_inertia_ff;
	float gravity_ff;
	float dyn_frct_ff; //torque(nm)/(rad/s)
	float stat_frct_ff;//breakway torque(nm)
	float trans_v_ff;//smoothing speed/(rad/s)
	float theta;
	float theta_d;
	float theta_dd;
	uint8_t tic;
	uint8_t cmd_rdy;
	bool new_traj_req;
	bool traj_cmplt;
	MinJerkTraj jerk_traj;
	ConstVel const_vel_traj;
	float chirp_start_freq, chirp_end_freq;
	float chirp_duration, chirp_amplitude, chirp_ramp_time, chirp_bias;
	LogChirpTraj chirp_traj;
}MotorTrajectory;

void motor_trajectory_init(MotorTrajectory* m_traj, uint8_t motor_id);
void advance_traj(MotorTrajectory* m_traj, MotorCommand* m_cmd);
void generate_traj_cmd(MotorTrajectory* m_traj, MotorCommand* m_cmd);
void reset_target_pos(MotorTrajectory* m_traj, MotorCommand* m_cmd);
void cancel_logchirp(MotorTrajectory* m_traj, MotorCommand* m_cmd);
bool logchirp_parameters_valid(float start_freq, float end_freq, float duration,
	float amplitude, float ramp_time, float bias, float dt);
bool logchirp_start(LogChirpTraj* tr, float start_freq, float end_freq, float duration,
	float amplitude, float ramp_time, float bias, float dt);
bool logchirp_step(LogChirpTraj* tr, float elapsed_seconds, float* torque);
void minjerk_start(MinJerkTraj* tr, float theta0, float thetaf, float T, float dt);
bool minjerk_step(MinJerkTraj* tr, float* theta, float* theta_dot, float* theta_ddot);
void constvel_start(ConstVel* tr, float theta0, float thetaf, float T, float dt);
bool constvel_step(ConstVel* tr, float* theta, float* theta_dot, float* theta_ddot);
float smooth_sign(float v, float v0);
float friction_ff(float v_des, float b_visc, float tau_breakaway, float v0);


#ifdef __cplusplus
}
#endif

#endif /* INC_TRAJECTORY_MANAGER_H_ */

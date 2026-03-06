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

#define TRAJ_LEN 1000

typedef struct{
	float theta0;//in radians
	float thetaf;//in radians
	float T; //In Seconds
	float t; //current time (locally), in seconds
	float dt; //timestep, in seconds
	bool active;
} MinJerkTraj;

typedef struct{
	uint8_t motor_id;
	uint8_t traj_mode;
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
}MotorTrajectory;


void advance_traj(MotorTrajectory* m_traj, MotorCommand* m_cmd);
void generate_traj_cmd(MotorTrajectory* m_traj, MotorCommand* m_cmd);
void minjerk_start(MinJerkTraj* tr, float theta0, float thetaf, float T, float dt);
bool minjerk_step(MinJerkTraj* tr, float* theta, float* theta_dot, float* theta_ddot);
float smooth_sign(float v, float v0);
float friction_ff(float v_des, float b_visc, float tau_breakaway, float v0);


#ifdef __cplusplus
}
#endif

#endif /* INC_TRAJECTORY_MANAGER_H_ */

/*
 * Copyright (C) GJ van Dam
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.c"
 * @author GJ van Dam
 * Obstacle avoidance method using RL and the aerodynamic interaction between obstacle and quadrotor
 */
// Include header file
#ifndef RL_OBSTACLE_AVOIDANCE_H
#include "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.h"
#endif

// Include standard libraries
#include <stdio.h>
#include "std.h"
#include <string.h>

// Include telemetry headers
#include "subsystems/datalink/telemetry.h"

// Include modules which will be used to retrieve measurements
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/actuators.h"
#include "boards/bebop/actuators.h"
#include "firmwares/rotorcraft/stabilization.h"

// Include other paparazzi modules
#include "generated/modules.h"

// Include module specific files
#include "log_to_file.h"

// Set pre-processor constants
#define RL_OBSTACLE_AVOIDANCE_LOG TRUE
#define RL_OBSTACLE_AVOIDANCE_TELEMETRY TRUE

// Declaration of global variables

// Declaration of local variables
static int32_t number_of_variables = 0;
struct timeval currentTime;
static int32_t start_time_seconds = 0;
static int32_t start_time_milliseconds = 0;

static int32_t timestep = 0;
static int32_t time_rl = 0;
static float body_acceleration_u = 0.0;
static float body_acceleration_v = 0.0;
static float body_acceleration_w = 0.0;
static float body_speed_u = 0.0;
static float body_speed_v = 0.0;
static float body_speed_w = 0.0;
static float enu_position_x = 0.0;
static float enu_position_y = 0.0;
static float enu_position_z = 0.0;
static float body_rate_p = 0.0;
static float body_rate_q = 0.0;
static float body_rate_r = 0.0;
static float body_attitude_phi = 0.0;
static float body_attitude_theta = 0.0;
static float body_attitude_psi = 0.0;
static uint16_t motor_speed_nw = 0;
static uint16_t motor_speed_ne = 0;
static uint16_t motor_speed_se = 0;
static uint16_t motor_speed_sw = 0;

static rl_variable list_of_variables_to_log[21] = {
        // Name, Type, Format, *pointer
        {"Timestep","int32_t","%d",&timestep},
        {"Time (milliseconds)","int32_t","%li",&time_rl},
        {"Body Acceleration u","float","%f",&body_acceleration_u},
        {"Body Acceleration v","float","%f",&body_acceleration_v},
        {"Body Acceleration w","float","%f",&body_acceleration_w},
        {"Body speed u","float","%f",&body_speed_u},
        {"Body speed v","float","%f",&body_speed_v},
        {"Body speed w","float","%f",&body_speed_w},
        {"ENU position x","float","%f",&enu_position_x},
        {"ENU position y","float","%f",&enu_position_y},
        {"ENU position z","float","%f",&enu_position_z},
        {"Body rate p","float","%f",&body_rate_p},
        {"Body rate q","float","%f",&body_rate_q},
        {"Body rate r","float","%f",&body_rate_r},
        {"Body attitude roll (phi)","float","%f",&body_attitude_phi},
        {"Body attitude pitch (theta)","float","%f",&body_attitude_theta},
        {"Body attitude yaw (psi)","float","%f",&body_attitude_psi},
        {"Motorspeed 1 (NW)","uint16_t","%d",&motor_speed_nw},
        {"Motorspeed 2 (NE)","uint16_t","%d",&motor_speed_ne},
        {"Motorspeed 3 (SE)","uint16_t","%d",&motor_speed_se},
        {"Motorspeed 4 (SW)","uint16_t","%d",&motor_speed_sw}
};

static void send_rl_variables(struct transport_tx *trans, struct link_device *dev)
{
    // When prompted, return all the telemetry variables
    pprz_msg_send_RL_OBSTACLE_AVOIDANCE(trans, dev, AC_ID,
                                        &timestep, &time_rl,
                                        &body_acceleration_u, &body_acceleration_v, &body_acceleration_w,
                                        &body_speed_u, &body_speed_v, &body_speed_w,
                                        &enu_position_x, &enu_position_y, &enu_position_z,
                                        &body_rate_p, &body_rate_q, &body_rate_r,
                                        &body_attitude_phi, &body_attitude_theta, &body_attitude_psi,
                                        &motor_speed_nw, &motor_speed_ne, &motor_speed_se, &motor_speed_sw);
}

/** Initialization function **/
void rl_obstacle_avoidance_init(void) {
    // Determine number of variables
    number_of_variables = sizeof(list_of_variables_to_log) / sizeof(rl_variable);

    // Register telemetery function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_OBSTACLE_AVOIDANCE, send_rl_variables);
}

/** Function called when the module is started, performs the following functions:
 * -> Create log file
 * -> Open log file
 * */
void rl_obstacle_avoidance_start(void){
    // If logging is turned on, create file
    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Create csv header line
        char csv_header_line[1000];
        int i;
        for (i=0; i < number_of_variables; i++){
            if(i>0){
                strcat(csv_header_line, ",");
            }
            strcat(csv_header_line, list_of_variables_to_log[i].name);
        }
        strcat(csv_header_line, "\n");

        // Write header line to file
        log_to_file_start(csv_header_line);
    }

    // Set start time in seconds
    gettimeofday(&currentTime, NULL);
    start_time_seconds = currentTime.tv_sec;
    start_time_milliseconds = currentTime.tv_usec;
}

void rl_obstacle_avoidance_update_measurements(void){
    // Update timestep
    timestep++;

    // Get time in milliseconds since the measurement has started
    gettimeofday(&currentTime, NULL);
    time_rl = (currentTime.tv_sec - start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 ;

    // Update accelerations in the body frame
    struct Int32Vect3 *body_accelerations = stateGetAccelBody_i();
    body_acceleration_u = ACCEL_FLOAT_OF_BFP(body_accelerations->x);
    body_acceleration_v = ACCEL_FLOAT_OF_BFP(body_accelerations->y);
    body_acceleration_w = ACCEL_FLOAT_OF_BFP(body_accelerations->z);

    // Update speeds in the body frame
    struct FloatVect3 *speed_ned = (struct FloatVect3 *)stateGetSpeedNed_f();
    struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
    struct FloatVect3 speed_body;
    float_rmat_transp_vmult(&speed_body, ned_to_body, speed_ned);
    body_speed_u = speed_body.x;
    body_speed_v = speed_body.y;
    body_speed_w = speed_body.z;

    // Update position in the ENU frame
    struct EnuCoor_f *enu_position_f = stateGetPositionEnu_f();
    enu_position_x = enu_position_f->x;
    enu_position_y = enu_position_f->y;
    enu_position_z = enu_position_f->z;

    // Update angular rates in the body frame
    struct FloatRates *body_rates_f = stateGetBodyRates_f();
    body_rate_p = body_rates_f->p;
    body_rate_q = body_rates_f->q;
    body_rate_r = body_rates_f->r;

    // Update euler angels in the body frame
    struct FloatEulers *ned_attitude_f = stateGetNedToBodyEulers_f();
    body_attitude_phi = ned_attitude_f->phi;
    body_attitude_theta = ned_attitude_f->theta;
    body_attitude_psi = ned_attitude_f->psi;

    // Update motor speeds
    motor_speed_nw = actuators_bebop.rpm_obs[0];
    motor_speed_ne = actuators_bebop.rpm_obs[1];
    motor_speed_se = actuators_bebop.rpm_obs[2];
    motor_speed_sw = actuators_bebop.rpm_obs[3];
//    motor_speed_nw = actuators[0];
//    motor_speed_ne = actuators[1];
//    motor_speed_se = actuators[2];
//    motor_speed_sw = actuators[3];
}


void rl_obstacle_avoidance_periodic(void) {
    // Update measurements
    rl_obstacle_avoidance_update_measurements();


    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Log measurements, states and actions to log file
        log_to_file_log_line(timestep, list_of_variables_to_log, number_of_variables);
    }

    if(RL_OBSTACLE_AVOIDANCE_TELEMETRY){
        // Send measurements, states and actions to telemetry
    }


}

void rl_obstacle_avoidance_turn_on(void){
    rl_obstacle_avoidance_rl_obstacle_avoidance_periodic_status = MODULES_START;
}

void rl_obstacle_avoidance_turn_off(void){
    rl_obstacle_avoidance_rl_obstacle_avoidance_periodic_status = MODULES_STOP;
}

/** Function called when the module is stopped, performs the following functions:
 * -> Close log file
 * */
void rl_obstacle_avoidance_stop(void){

    // If logging is turned on, close file
    if(RL_OBSTACLE_AVOIDANCE_LOG){
        log_to_file_stop();
    }
}
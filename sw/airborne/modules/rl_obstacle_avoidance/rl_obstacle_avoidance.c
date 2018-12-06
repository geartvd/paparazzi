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
#include <errno.h>

// Include telemetry headers
#include "subsystems/datalink/telemetry.h"

// Include modules which will be used to retrieve measurements
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/actuators.h"
#include "boards/bebop/actuators.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"

// Include other paparazzi modules
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h" // dl_buffer

// Include module specific files
#include "log_to_file.h"

// Set pre-processor constants
#define RL_OBSTACLE_AVOIDANCE_LOG TRUE
#define RL_OBSTACLE_AVOIDANCE_TELEMETRY TRUE

// Declaration of global variables
float rl_obstacle_avoidance_filter_cutoff = 3.0;

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
static uint16_t motor_speed_nw_cmd = 0;
static uint16_t motor_speed_ne_cmd = 0;
static uint16_t motor_speed_se_cmd = 0;
static uint16_t motor_speed_sw_cmd = 0;
static int32_t flight_status = 0;
static float F_ext_onboard_est = 0.0;
static float body_acceleration_w_filt = 0.0;
static float F_thrust_est = 0.0;

//static float filt_tau;
//static float filt_sample_time;
static Butterworth4LowPass filt_accel_body[3];
static Butterworth4LowPass filt_motor_speed[4];
static Butterworth4LowPass filt_body_rate[4];
#ifndef USE_NPS
static float estimated_k_over_m[4] = {3.7352494607033004E-06, 3.7352494607033004E-06, 3.7352494607033004E-06, 3.7352494607033004E-06};
static float estimated_accel_bias[3] = {0.0, 0.0, -15.191601570000005};
static int set_estimated_k_over_m = true;
static int set_estimated_accel_bias = true;
#endif

// RL variables
#define RL_DISCR_BINS_F_EXT 5
#define RL_DISCR_BINS_F_EXT_ROLLING 5

static rl_state current_state;
static float discr_state_F_ext_bounds[5] = {-1.5, -1.2, -0.9, -0.6, -0.3};
static float discr_state_F_ext_rolling_bounds[5] = {-1.5, -1.2, -0.9, -0.6, -0.3};
static int current_policy[5][5] = {
        {1, 1, 1, 1, 1},
        {1, 1, 2, 1, 1},
        {1, 2, 2, 1, 1},
        {1, 2, 2, 1, 1},
        {1, 1, 1, 1, 1}
};


// Variables used to simulate the ground effect
#define RL_NUMBER_OF_MEAUSUREMENTS 150432
static FILE *csv_measurements = NULL;
//static F_ext_measurement measurements[150500];
static float F_ext_measurements_z[RL_NUMBER_OF_MEAUSUREMENTS];
static float F_ext_measurements_F_ext[RL_NUMBER_OF_MEAUSUREMENTS];
static int random_in_range(int min, int max);
static int rl_intervention_save = false;
static float F_ext_rolling_mean;

static rl_variable list_of_variables_to_log[30] = {
        // Name, Type, Format, *pointer
        {"Timestep","int32_t","%d",&timestep},
        {"Time (milliseconds)","int32_t","%li",&time_rl},
        {"Flight status","int32_t","%d",&flight_status},
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
        {"Motorspeed 4 (SW)","uint16_t","%d",&motor_speed_sw},
        {"Motorspeed 1 (NW)_cmd","uint16_t","%d",&motor_speed_nw_cmd},
        {"Motorspeed 2 (NE)_cmd","uint16_t","%d",&motor_speed_ne_cmd},
        {"Motorspeed 3 (SE)_cmd","uint16_t","%d",&motor_speed_se_cmd},
        {"Motorspeed 4 (SW)_cmd","uint16_t","%d",&motor_speed_sw_cmd},
        {"F_ext_onboard_est","float","%f",&F_ext_onboard_est},
        {"F_ext_rolling_mean", "float", "%f",&F_ext_rolling_mean},
        {"body_acceleration_w_filt","float","%f",&body_acceleration_w_filt},
        {"F_thrust_est","float","%f",&F_thrust_est},
};

// Define static functions
#ifndef USE_NPS
// Real life
static float estimate_F_ext_onboard(void);
#else
// Simulator
static float estimate_F_ext_from_measurements(float height);
static void parseCSVmeasurements(void);
static float binary_search(float sorted_list[], int low, int high, float element);
#endif

static int rl_obstacle_avoidance_get_action(rl_state state);
static void rl_obstacle_avoidance_state_estimator(void);
static void rl_obstacle_avoidance_perform_action(int action);
static void rl_obstacle_avoidance_parse_uplink(void);

static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);

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
                                        &motor_speed_nw, &motor_speed_ne, &motor_speed_se, &motor_speed_sw,
                                        &F_ext_onboard_est);
}

/** Initialization function **/
void rl_obstacle_avoidance_init(void) {
    // Determine number of variables
    number_of_variables = sizeof(list_of_variables_to_log) / sizeof(rl_variable);

    // Register telemetery function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_OBSTACLE_AVOIDANCE, send_rl_variables);

#ifdef USE_NPS // Simulation
    // Load CSV measurements
    parseCSVmeasurements();
#endif
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

    // Initialize butterworth filters
    float filt_tau = 1.0 / (2.0 * M_PI * rl_obstacle_avoidance_filter_cutoff);
    float filt_sample_time= 1.0 / PERIODIC_FREQUENCY;

    // Filtered body accelerations
    for (int8_t i = 0; i < 3; i++) {
        init_butterworth_4_low_pass(&filt_accel_body[i], filt_tau, filt_sample_time, 0.0);
    }

    // Filtered RPM
    for (int8_t i = 0; i < 4; i++) {
        init_butterworth_4_low_pass(&filt_motor_speed[i], filt_tau, filt_sample_time, 0.0);
    }

    // Filtered body rates
    for (int8_t i = 0; i < 3; i++) {
        init_butterworth_4_low_pass(&filt_body_rate[i], filt_tau, filt_sample_time, 0.0);
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

    // Update filtered accelerations in the body frame
    update_butterworth_4_low_pass(&filt_accel_body[0], body_acceleration_u);
    update_butterworth_4_low_pass(&filt_accel_body[1], body_acceleration_v);
    update_butterworth_4_low_pass(&filt_accel_body[2], body_acceleration_w);
    body_acceleration_w_filt = filt_accel_body[2].lp2.o[0];

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

    // Update filtered angular rates in the body frame
    update_butterworth_4_low_pass(&filt_body_rate[0], body_rate_p);
    update_butterworth_4_low_pass(&filt_body_rate[1], body_rate_q);
    update_butterworth_4_low_pass(&filt_body_rate[2], body_rate_r);

    // Update euler angels in the body frame
    struct FloatEulers *ned_attitude_f = stateGetNedToBodyEulers_f();
    body_attitude_phi = ned_attitude_f->phi;
    body_attitude_theta = ned_attitude_f->theta;
    body_attitude_psi = ned_attitude_f->psi;

#ifndef USE_NPS
    // Update motor speeds
    motor_speed_nw = actuators_bebop.rpm_obs[0];
    motor_speed_ne = actuators_bebop.rpm_obs[1];
    motor_speed_se = actuators_bebop.rpm_obs[2];
    motor_speed_sw = actuators_bebop.rpm_obs[3];

    // Update commanded motor speeds
    motor_speed_nw_cmd = actuators_bebop.rpm_ref[0];
    motor_speed_ne_cmd = actuators_bebop.rpm_ref[1];
    motor_speed_se_cmd = actuators_bebop.rpm_ref[2];
    motor_speed_sw_cmd = actuators_bebop.rpm_ref[3];

    // Update filtered motor speeds
    for (int8_t i = 0; i < 4; i++) {
        update_butterworth_4_low_pass(&filt_motor_speed[i], actuators_bebop.rpm_obs[i]);
    }
#endif

    // Estimate F_ext
#ifdef USE_NPS
    // Simulation, so use
    F_ext_onboard_est = estimate_F_ext_from_measurements(enu_position_z);
#else
    // Onboard estimate
    F_ext_onboard_est = estimate_F_ext_onboard();
#endif
}

#ifndef USE_NPS
static float estimate_F_ext_onboard(void){
    float F_ext = 0.0;

    if(set_estimated_k_over_m && set_estimated_accel_bias) {
        // Estimate thrust produced by rotors
        F_thrust_est = 0.0;
        for (int8_t i = 0; i < 4; i++) {
            F_thrust_est =
                    F_thrust_est + estimated_k_over_m[i] * powf((filt_motor_speed[i].lp2.o[0] * 2 * M_PI / 60), 2);
        }
        F_ext = (filt_accel_body[2].lp2.o[0] - estimated_accel_bias[2]) \
                 + (filt_body_rate[0].lp2.o[0] * body_speed_v) \
                 - (filt_body_rate[1].lp2.o[0] * body_speed_u) \
                 - 9.81 * cos(body_attitude_phi) * cos(body_attitude_theta) \
                 + F_thrust_est;
//        printf("(%f)+(%f*%f)-(%f*%f)-9.81*%f*%f+%f\n", \
//                filt_accel_body[2].lp2.o[0], \
//                filt_body_rate[0].lp2.o[0], body_speed_v, \
//                filt_body_rate[1].lp2.o[0], body_speed_u, \
//                cos(body_attitude_phi), cos(body_attitude_theta), \
//                F_thrust_est);
    }
    return F_ext;
}
#endif


void rl_obstacle_avoidance_periodic(void) {
    int next_action;

    // Update measurements
    rl_obstacle_avoidance_update_measurements();

    // Get state
    rl_obstacle_avoidance_state_estimator();

    if((flight_status >= 50) && (flight_status < 80)) {
        // Prepare next timestep, get next action from policy
        next_action = rl_obstacle_avoidance_get_action(current_state);
//        printf("[%d][%d] -> %d\n", current_state.discr_F_ext, current_state.discr_F_ext_rolling, next_action);

        // Check safety of action

        // Check need to abort

        // Perform action
        rl_obstacle_avoidance_perform_action(next_action);
    }

    if(RL_OBSTACLE_AVOIDANCE_LOG){
        // Log measurements, states and actions to log file
        log_to_file_log_line(timestep, list_of_variables_to_log, number_of_variables);
    }


}

void rl_obstacle_avoidance_state_estimator(void){
    float F_ext;
    float F_ext_rolling_sum = 0.0;

    int F_ext_discr;
    int F_ext_rolling_discr;
    int8_t i;

    // Get F_ext (set by update_measurements)
    F_ext = F_ext_onboard_est;

    // Update rolling, shift all elements in the array 1 place to the right
    for (int8_t i=RL_ROLLING_WINDOW_SIZE-1; i > 0; i--){
        current_state.F_ext_rolling[i] = current_state.F_ext_rolling[i-1];
        F_ext_rolling_sum = F_ext_rolling_sum + current_state.F_ext_rolling[i];

    }

    // Replace the first value
    current_state.F_ext_rolling[0]= F_ext;
    F_ext_rolling_sum = F_ext_rolling_sum + F_ext;
    F_ext_rolling_mean = F_ext_rolling_sum / RL_ROLLING_WINDOW_SIZE;

//    for (int8_t i=0; i < RL_ROLLING_WINDOW_SIZE; i++){
//        printf("%f +",current_state.F_ext_rolling[i]);
//    }
//    printf("= %f -> %f\n", F_ext_rolling_sum, F_ext_rolling_mean);

    // Discretize F_ext
    F_ext_discr = RL_DISCR_BINS_F_EXT-1;// Maximum value
    for (i=0; i < RL_DISCR_BINS_F_EXT; i++){
        if(F_ext < discr_state_F_ext_bounds[i+1]){
            F_ext_discr = i;
            break;
        }
    }


    // Discretize F_ext_rolling
    F_ext_rolling_discr = RL_DISCR_BINS_F_EXT_ROLLING-1; // Maximum value
    for (i=0; i < RL_DISCR_BINS_F_EXT; i++){
        if(F_ext_rolling_mean < discr_state_F_ext_rolling_bounds[i+1]){
//            printf("if %f < %f -> %d\n", F_ext_rolling_mean, discr_state_F_ext_rolling_bounds[i+1], i);
            F_ext_rolling_discr = i;
            break;
        };
    }

    // Save to state
    current_state.discr_F_ext = F_ext_discr;
    current_state.discr_F_ext_rolling = F_ext_rolling_discr;

}

int rl_obstacle_avoidance_get_action(rl_state state){
    int action;

   // Get action from policy
    action = current_policy[state.discr_F_ext][state.discr_F_ext_rolling];
    return action;
}

void rl_obstacle_avoidance_perform_action(int action){
    if((action ==2) && (rl_intervention_save == false)){
        printf("intervention at height %f! F_ext=%f [%d], F_ext_rolling=%f [%d]\n",enu_position_z, F_ext_onboard_est, current_state.discr_F_ext, \
        F_ext_rolling_mean, current_state.discr_F_ext_rolling);
        rl_intervention_save = true;
    }
}

int rl_obstacle_avoidance_save(void){
    return rl_intervention_save;
}

void rl_obstacle_avoidance_save_concluded(void){
    rl_intervention_save = false;
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

/** Functon used to set and store the flight status:
 * */
void rl_obstacle_avoidance_flight_status(int status){
    flight_status = status;
}

#ifdef USE_NPS

/** Function used to simulation F_ext, based on previous measurements
 * */
float estimate_F_ext_from_measurements(float height){
    float F_ext_estimate;
    int height_id;

    // Search the current height in the sorted list of measurements

    height_id = binary_search(F_ext_measurements_z, 0,RL_NUMBER_OF_MEAUSUREMENTS, height);
    if(height_id<0){
        height_id = 0;
    }
    if(height_id>(RL_NUMBER_OF_MEAUSUREMENTS-1)){
        height_id = RL_NUMBER_OF_MEAUSUREMENTS-1;
    }

    F_ext_estimate = F_ext_measurements_F_ext[height_id];

//    printf("%f -> %d -> %f\n",height, height_id, F_ext_estimate);

    return F_ext_estimate;  
}


/** Function used to parse the measurements csv file
 * */
void parseCSVmeasurements(void){
    char buf[120];
    char *item;
    int32_t reccount = 0;

    csv_measurements = fopen("/home/geart/Measurements/ground_F_ext_c.csv","r");

    if (csv_measurements == NULL) {
        printf("Error opening measurement file: %s\n", strerror(errno));
    }

    // Loop through file
    while (fgets(buf,120,csv_measurements)) {

        // Step
        item = strtok(buf, ",");

        // F_ext
        item = strtok(NULL, ",");
        F_ext_measurements_F_ext[reccount] = atof(item);

        // z
        item = strtok(NULL, " ");
        F_ext_measurements_z[reccount] = atof(item);

        reccount++;
    }

    // Close file
    fclose(csv_measurements);
}


/** Binary search algorithm
 * */

float binary_search(float sorted_list[], int low, int high, float element)
{

    if (high < low){
        int random_int = random_in_range(high-10,low+10);
        return random_int;
    }
    int middle = low + (high - low)/2;
    if (element < sorted_list[middle])
        return binary_search(sorted_list, low, middle-1, element);
    else if (element > sorted_list[middle])
        return binary_search(sorted_list, middle+1, high, element);
    else
        return middle;
}

int random_in_range(int min, int max){
    return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

#endif

void rl_obstacle_avoidance_parse_uplink(uint8_t msg_id, ){
    // dl_buffer

}
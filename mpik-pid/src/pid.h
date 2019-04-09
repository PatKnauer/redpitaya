/**
 * @brief Red Pitaya PID Controller
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#ifndef __PID_H
#define __PID_H

#include "main.h"

int pid_init(void);
int pid_exit(void);

int pid_update(rp_app_params_t *params);
int AO_update(rp_app_params_t *params);		//patrick - added AO function for backend
int piezo_drive(rp_app_params_t *params);	//patrick - added piezo drive function to chose up or down
int piezo_pulse(rp_app_params_t *params);	//patrick - added piezo pulse function to send pulse
int laser_locked(rp_app_params_t *params);	//patrick - added function to laser lock
int pid_update_meas_output(rp_osc_meas_res_t *ch_meas, int channel); // bar graph -------------- Fenske
float pid_min_intensity(int channel); // compares min_intensity input to threshold  -------------- Fenske

#endif // __PID_H

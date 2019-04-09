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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h> // for mmap
#include <fcntl.h> // for opening files
#include <unistd.h> //patrick - needed for usleep()

#include "redpitaya/rp.h"

#include "pid.h"
#include "fpga_pid.h"

// Variables
void* map_ams = (void*)(-1);

float dac_kp_1 = 425; // ------------------- Fenske: Value for Input == Output 
float norm_int = (1/0.477); // value to achieve a normation of Ki to: 1/ms
float norm_diff = 140.3; // value to achieve a normation of Kd to: 1ns
int dac_dc_offset[] = {-150, -105};
float max_cnt = 8191;
float old_int_limit[NUM_OF_PIDS] = {1, 1, 1, 1}; // is needed to compare the integrator limit settings

FILE *fd; // new file for saving pid-settings ------------------ Fenske
int idx; // count variable
int no_settings = (PARAMS_PER_GEN*2)+1 + (NUM_OF_PIDS*PARAMS_PER_PID) +4; //number of AWG+PID settings (+1 for AWG_REFRESH, +4 for saving the offset and threshold values too)
float min_intens_threshold[] = {0, 0}; // local copie of threshold value to which the XADC value is compared to
int threshold_reached[] = {0, 0}; // flag if threshold is reached (for later)
float xadc_offset[] = {0, 0}; // offsetvalue of XADC measured at pid_init()
int old_setpoint[] = {0, 0, 0, 0}; // saves the old_setpoint value

/**
 * GENERAL DESCRIPTION:
 *
 * The code below sets the PID parameters to the user-specified values.
 * A direct translation from GUI PID parameters to the FPGA registers.
 *
 * The PID Controller algorithm itself is implemented in FPGA.
 * There are 4 independent PID controllers, connecting each input (IN1, IN2)
 * to each output (OUT1, OUT2):
 *
 *                 /-------\       /-----------\
 *   IN1 -----+--> | PID11 | ------| SUM & SAT | ---> OUT1
 *            |    \-------/       \-----------/
 *            |                            ^
 *            |    /-------\               |
 *            ---> | PID21 | ----------    |
 *                 \-------/           |   |
 *                                     |   |
 *                                     |   |
 *                                     |   |
 *                 /-------\           |   |
 *            ---> | PID12 | --------------
 *            |    \-------/           |
 *            |                        ˇ
 *            |    /-------\       /-----------\
 *   IN2 -----+--> | PID22 | ------| SUM & SAT | ---> OUT2
 *                 \-------/       \-----------/
 *
 */


/*----------------------------------------------------------------------------------*/
/** @brief Initialize PID Controller module
 *
 * A function is intended to be called within application initialization. It's purpose
 * is to initialize PID Controller module and to calculate maximal voltage, which can be
 * applied on DAC device on individual channel.
 *
 * @retval     -1 failure, error message is reported on standard error
 * @retval      0 successful initialization
 */

int pid_init(void)
{
    if(fpga_pid_init() < 0) {
        return -1;
    }
    system("cd /opt/redpitaya/www/apps/ptb-pid/\n rw"); // should make the folder "ptb-pid" writeable ------ Fenske CHANGE for pid2!!!
    /* 	The C library function int system(const char *command) passes the command name
    	or program name specified by command to the host environment to be executed
    	by the command processor and returns after the command has been completed. */
    //  (necessary for saving the settings in a .txt-file)

    if (rp_Init() != RP_OK) { // Init API functions!!!
         fprintf(stderr, "Red Pitaya API init failed!\n");
         return EXIT_FAILURE;
    }
    return 0;
}


/*----------------------------------------------------------------------------------*/
/** @brief Cleanup PID COntroller module
 *
 * A function is intended to be called on application's termination. The main purpose
 * of this function is to release allocated resources...
 *
 * @retval      0 success, never fails.
 */
int pid_exit(void)
{
    fpga_pid_exit();

    return 0;
}


/*----------------------------------------------------------------------------------*/
/**
 * @brief Update PID Controller module towards actual settings.
 *
 * A function is intended to be called whenever one of the following settings on each PID
 * sub-controller is modified:
 *    - Enable
 *    - Integrator reset
 *    - Set-point
 *    - Kp
 *    - Ki
 *    - Kd
 *
 * @param[in] params  Pointer to overall configuration parameters
 * @retval -1 failure, error message is repoted on standard error device
 * @retval  0 succesful update
 */
int pid_update(rp_app_params_t *params)
{
    int i;

    pid_param_t pid[NUM_OF_PIDS] = {{ 0 }};
    uint32_t ireset = 0;

    for (i = 0; i < NUM_OF_PIDS; i++) {
        /* PID enabled? */
        if (params[PID_11_ENABLE + i * PARAMS_PER_PID].value == 1) {
        	pid[i].gain = (int)((params[PID_11_GAIN + i * PARAMS_PER_PID].value)*128);  // Fenske (*128 to get integer-value)
        	pid[i].setpoint = (int)(((params[PID_11_SP + i * PARAMS_PER_PID].value)*max_cnt)); // + dac_dc_offset[i]); // Fenske
        	old_setpoint[i] = pid[i].setpoint;
        	pid[i].kp = (int)((params[PID_11_KP + i * PARAMS_PER_PID].value)*dac_kp_1);  // Fenske
            pid[i].ki = (int)((params[PID_11_KI + i * PARAMS_PER_PID].value)*norm_int);
            pid[i].kd = (int)((params[PID_11_KD + i * PARAMS_PER_PID].value)*norm_diff);
            pid[i].limit_up = (int)((params[PID_11_LIMIT_UP + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].limit_low = (int)((params[PID_11_LIMIT_LOW + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].int_limit = (int)((params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].kii = (int)((params[PID_11_KII + i * PARAMS_PER_PID].value)*norm_int);
            pid[i].count_ki = (int)((params[PID_11_COUNT_KI + i * PARAMS_PER_PID].value)-1); // -1 so that "/x" can be set in the user interface

            if(old_int_limit[i] > params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value) { // reset integrator if int_limit is set
            	g_pid_reg->pid[i].ki = 0;
            }
            old_int_limit[i] = params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value;
        }
        else if (params[PID_11_ENABLE + i * PARAMS_PER_PID].value != 1) {  // added by Fenske (to reset the output if PID is not enabled -------
            ireset |= (1 << i);
        }

        g_pid_reg->pid[i].gain = pid[i].gain;
        g_pid_reg->pid[i].setpoint = pid[i].setpoint;
        g_pid_reg->pid[i].kp = pid[i].kp;
        g_pid_reg->pid[i].ki = pid[i].ki;
        g_pid_reg->pid[i].kd = pid[i].kd;
        g_pid_reg->pid[i].limit_up = pid[i].limit_up; // ------------------------------------------------ Fenske
        g_pid_reg->pid[i].limit_low = pid[i].limit_low; // ---------------------------------------------- Fenske
        g_pid_reg->pid[i].int_limit = pid[i].int_limit; // Fenske
        g_pid_reg->pid[i].kii = pid[i].kii;
        g_pid_reg->pid[i].count_ki = pid[i].count_ki;

        if (params[PID_11_RESET + i * PARAMS_PER_PID].value == 1) {
            ireset |= (1 << i);
        }

        /* added by Fenske for automatic Integrator reset-------------------------------------------------*/
        if (params[PID_11_ARESET + i * PARAMS_PER_PID].value == 1) {
            ireset |= (1 << (i+NUM_OF_PIDS));
        }
    }
    
    g_pid_reg->configuration = ireset;

    // Offset settings go to FPGA
    g_pid_reg->out_1_offset = (int)((params[OUT_1_OFFSET].value)*max_cnt); // ------------------- Fenske
    g_pid_reg->out_2_offset = (int)((params[OUT_2_OFFSET].value)*max_cnt); // ------------------- Fenske

    // Threshold for min_intensity goes to local parameter needed by "pid_min_intensity()"
    min_intens_threshold[0] = params[MIN_I_THRESHOLD_1].value; // --------------- Fenske
    min_intens_threshold[1] = params[MIN_I_THRESHOLD_2].value; // --------------- Fenske

    // Save settings ---------------------------------------------------------------------------- Fenske
    // (for creating files it is necessary to mount the folder on the SD-card writable! see pid_init() )
    if (params[SAVE_SETTINGS].value != 0)
    {
    	switch((int) params[SAVE_SETTINGS].value)
    	{
    		case 1: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings1.txt", "w"); break;
    		case 2: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings2.txt", "w"); break;
    		case 3: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings3.txt", "w"); break;
    		default: break;
    	}
    	if(fd != NULL) {
    		for(idx = 0; idx < no_settings + 27; idx++){		//+27 for new settings
			if(idx < no_settings){
    			fprintf(fd, "%f\n", params[PARAMS_AWG_PARAMS+idx].value);
			}
			else{
			fprintf(fd, "%f\n", params[idx + 54].value);		//starts at 72(no_settings)+54=126
			}
    		}
    	}
    	fclose(fd);
    	params[SAVE_SETTINGS].value = 0;
    }
    // Load settings ---------------------------------------------------------------------------- Fenske
    if (params[LOAD_SETTINGS].value != 0)
    {
        switch((int) params[LOAD_SETTINGS].value)
        {
        	case 1: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings1.txt", "r"); break;
        	case 2: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings2.txt", "r"); break;
        	case 3: fd = fopen("/opt/redpitaya/www/apps/mpik-pid/settings3.txt", "r"); break;
        	default: break;
        }
        if(fd != NULL) {
        	for(idx = 0; idx < no_settings + 27; idx++){
			if(idx < no_settings){
        		fscanf(fd, "%f\n", &params[PARAMS_AWG_PARAMS+idx].value);
			}
			else{
			fscanf(fd, "%f\n", &params[idx + 54].value);
			}
        	}
        }
        fclose(fd);
        params[LOAD_SETTINGS].value = 0;
        pid_update(&params[0]); // load saved and loaded params into fpga
    }
    return 0;
}

// Reads measure values for P, I, D and Output from FPGA
int pid_update_meas_output(rp_osc_meas_res_t *ch_meas, int channel)  // bar graph data ---------- Fenske
{
	ch_meas->p = g_pid_reg->meas[channel-1].p;
	ch_meas->i = g_pid_reg->meas[channel-1].i;
	ch_meas->d = g_pid_reg->meas[channel-1].d;
	ch_meas->o = g_pid_reg->meas[channel-1].o;
	ch_meas->min_intensity = pid_min_intensity(channel);
	return 0;
}

int pid_min_intensity_offset(int channel)
{
	// gets offset value of XADCs 2 and 3 and stores them to xadc_offset
	switch(channel) {
			case 1: rp_AIpinGetValue(2, &xadc_offset[0]); break;
			case 2: rp_AIpinGetValue(3, &xadc_offset[1]); break;
			default: break;
	}
	return 0;
}

// Reads measure value of XADC and compares it to the threshold given in the interface ------------------ Fenske
float pid_min_intensity(int channel)
{
	float intensity = 0;
	int count = 10;

	switch(channel) {
		case 1:{ // XADC2
			rp_AIpinGetValue(2, &intensity); // read XADC2 value
			intensity = intensity - xadc_offset[0]; // subtract the offset
			if(intensity >= min_intens_threshold[0]){ // compare the intensity to the threshold
				if(old_setpoint[0] == (g_pid_reg->pid[0].setpoint)) threshold_reached[0] = 0; // if everything is fine -> do nothing
				else g_pid_reg->pid[0].setpoint = old_setpoint[0]; // if the threshold is reached in this moment, reset the setpoint to old value
			}
			if((old_setpoint[0] < max_cnt) && (min_intens_threshold[0] < 1) && (intensity < min_intens_threshold[0])){
				if(g_pid_reg->pid[0].setpoint == max_cnt){ // if one limit is reached, reset and search in the other direction
					g_pid_reg->pid[0].setpoint = 0;
					count = -10;
				}
				else if(g_pid_reg->pid[0].setpoint == -max_cnt){
					g_pid_reg->pid[0].setpoint = 0;
					count = 10;
				}
				else g_pid_reg->pid[0].setpoint = g_pid_reg->pid[0].setpoint + count; // is (intensity < min_intens_threshold) increase the error signal by increasing the setpoint
			}
			break;
		}
		case 2:{ // XADC3
			rp_AIpinGetValue(3, &intensity); // read XADC3 value
			intensity = intensity - xadc_offset[1]; // subtract the offset
			if(intensity >= min_intens_threshold[1]){ // compare the intensity to the threshold
				if(old_setpoint[3] == (g_pid_reg->pid[3].setpoint)) threshold_reached[1] = 0; // if everything is fine -> do nothing
				else g_pid_reg->pid[3].setpoint = old_setpoint[3]; // if the threshold is reached in this moment, reset the setpoint to old value
			}
			if((old_setpoint[3] < max_cnt) && (min_intens_threshold[1] < 1) && (intensity < min_intens_threshold[1])){
				if(g_pid_reg->pid[3].setpoint == max_cnt){ // if one limit is reached, reset and search in the other direction
					g_pid_reg->pid[3].setpoint = 0;
					count = -10;
				}
				else if(g_pid_reg->pid[3].setpoint == -max_cnt){
					g_pid_reg->pid[3].setpoint = 0;
					count = 10;
				}
				else g_pid_reg->pid[3].setpoint = g_pid_reg->pid[3].setpoint + count; // is (intensity < min_intens_threshold) increase the error signal by increasing the setpoint
			}
			break;
		}
		default: break; // threshold_reached = 0; break;
	}
	return intensity; // threshold_reached;
}

//ALL ADDITIONS FOLLOWING BY PATRICK

//patrick - slow piezo control and AO signals
//todo: while writing variables maybe lock the pthread

int AO_update(rp_app_params_t *params)
{	
	//defining voltage parameters
	float vol0 = params[AO_VOL_PIN0].value;
	float vol1 = params[AO_VOL_PIN1].value;
	float vol2 = params[AO_VOL_PIN2].value;
	float vol3 = params[AO_VOL_PIN3].value;

	//pin0
	if(params[AO_ENABLE_PIN0].value == 1) {
		rp_AOpinSetValue(0, vol0);
	}
	else if(params[AO_ENABLE_PIN0].value == 0) {
		rp_AOpinSetValue(0, 0);
	}
	//pin1
	if(params[AO_ENABLE_PIN1].value == 1) {
		rp_AOpinSetValue(1, vol1);
	}
	else if(params[AO_ENABLE_PIN1].value == 0) {
		rp_AOpinSetValue(1, 0);
	}
	//pin2
	if(params[AO_ENABLE_PIN2].value == 1) {
		rp_AOpinSetValue(2, vol2);
	}
	else if(params[AO_ENABLE_PIN2].value == 0) {
		rp_AOpinSetValue(2, 0);
	}
	//pin3
	if(params[AO_ENABLE_PIN3].value == 1) {
		rp_AOpinSetValue(3, vol3);
	}
	else if(params[AO_ENABLE_PIN3].value == 0) {
		rp_AOpinSetValue(3, 0);
	}
	return 0;
}

//patrick - piezo drive logic that automatically steers a piezo with AO

//function to determine the direction
int piezo_drive(rp_app_params_t *params)
{
	//defining variables/parameters
	float trsh_up = params[AO_TRSH_UP].value;
	float trsh_down = params[AO_TRSH_DOWN].value;
	float ch1out = params[MEAS_O_CH1].value;
	float nop = params[MANUAL_NOP].value;
//	float ch2out = params[MEAS_O_CH2].value;

//manual mode

	if((params[MANUAL_GO].value == 1) || (params[PIEZO_MANUAL].value == 1)){
		if(params[MANUAL_DIR].value == 1){		//1 is up
		rp_AOpinSetValue(0, 1.8);
		}
		else if(params[MANUAL_DIR].value == 0){		//0 is down
		rp_AOpinSetValue(0, 0);
		}
		if(params[MANUAL_GO].value == 1){
			if((params[MANUAL_TIMER].value < nop) && (params[PROD_PULSE].value == 0)){	
			params[PROD_PULSE].value = 1;
			params[MANUAL_TIMER].value = params[MANUAL_TIMER].value + 1;
			}
			else if(params[MANUAL_TIMER].value >= nop){
			params[MANUAL_TIMER].value = 0;
			params[MANUAL_GO].value = 0;
			}
		}
	}

//automatic mode

	else{
		if(ch1out > trsh_up){	//both in V		
			//drive upwards - high ttl signal
			rp_AOpinSetValue(0, 1.8);
			if(params[LL].value == 1){
				params[PROD_PULSE].value = 1;		//makes pulse start
			}
		}
		else if(ch1out < trsh_down){			
			//drive downwards - low ttl signal
			rp_AOpinSetValue(0, 0);
			if(params[LL].value == 1){
				params[PROD_PULSE].value = 1;		//makes pulse start
			}
		}
	}
	return 1;
}

//patrick - pulse function
int piezo_pulse(rp_app_params_t *params)
{	
	int len_ms = params[PULSE_LEN].value;
	//calibration
	float len = len_ms/11.9188 - 2.1542/11.9188;		//some pulse lenghts might be up to 10ms short because the len value is compared to integers, for pulse lenghts that correspond to whole numbers we get correct values

//trying to send fast pulses

if(params[PROD_PULSE].value == 1 && params[SLOW_PULSES].value == 0){
	int timer = 0;
		while(timer < params[MANUAL_NOP].value){
		rp_AOpinSetValue(1, 1.8);
		usleep(1);
		rp_AOpinSetValue(1, 0);
		usleep(1);
		timer++;
		}
	params[PROD_PULSE].value = 0;
	params[LASER_LOCK_VALUE].value = 0;
	params[MANUAL_TIMER].value = 0;
	params[MANUAL_GO].value = 0;
}

//slow pulses (for when the piezo doesn't react in time)
if(params[PROD_PULSE].value == 1){				//pulse needs to finish even when threshold is not met
	if(params[PULSE_TIMER].value < len/2){
	rp_AOpinSetValue(1, 0);
	params[PULSE_TIMER].value = params[PULSE_TIMER].value+1;
	}
	else if(params[PULSE_TIMER].value < len){
	rp_AOpinSetValue(1, 1.8);
	params[PULSE_TIMER].value = params[PULSE_TIMER].value+1;
	}
	else if(params[PULSE_TIMER].value >= len){
	rp_AOpinSetValue(1, 0);
	params[PULSE_TIMER].value = 0;
	params[PROD_PULSE].value = 0;
	params[LASER_LOCK_VALUE].value = 0;	//after producing a pulse the lock has to be certified again
	}
   }
	return 0;
}

//patrick - simple way to check if we are locked - could be improved with a standart deviation e.g.
int laser_locked(rp_app_params_t *params)
{
	params[LL_TIMER].value = params[LL_TIMER].value+1;	//increases when function is called (first time to 0)
	float ch1out = params[MEAS_O_CH1].value;
	float value0 = params[VALUE0].value;			//value to compare with
	int timer = params[LL_TIMER].value;			//should increase everytime function is called
	float dev = (ch1out - value0)*(ch1out - value0);	//deviation
	float all_dev = params[DEV_IN_V].value * params[DEV_IN_V].value;	//allowed deviation
	float x = params[TIMERLIMIT].value;

	if(timer == 0){
	params[VALUE0].value = ch1out;
	}
	else if(timer < x){
		if(dev < all_dev){
		params[LASER_LOCK_VALUE].value = params[LASER_LOCK_VALUE].value + 1;	//tracks locking
		}
		else{
		params[LASER_LOCK_VALUE].value = 0;					//resets locking value
		}
	}
	else if(timer >= x){
		if(dev < all_dev){
		params[LASER_LOCK_VALUE].value = params[LASER_LOCK_VALUE].value + 1;
		}
		else{
		params[LASER_LOCK_VALUE].value = 0;
		}
		params[LL_TIMER].value = -1;		//reset timer value to -1
	}
	if(params[LASER_LOCK_VALUE].value >= params[DEV_CHECKS].value){	//increase to check for higher stability
	params[LL].value = 1;
	params[LASER_LOCK_VALUE].value = params[DEV_CHECKS].value;
	}
	else{
	params[LL].value = 0;
	}
	return 0;
}

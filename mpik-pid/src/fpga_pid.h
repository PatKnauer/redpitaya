/**
 * @brief Red Pitaya PID FPGA controller.
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#ifndef _FPGA_PID_H_
#define _FPGA_PID_H_

#include <stdint.h>

/** @defgroup fpga_pid_h PID Controller
 * @{
 */

/** Base PID FPGA address */
#define PID_BASE_ADDR 0x40600000 // changed to FREE Space in MMAP -----------------------------Fenske
/** Base PID FPGA core size */
#define PID_BASE_SIZE 0x100

/** Number of PID controllers */
#define NUM_OF_PIDS 4 //----------------------------------- changed by Fenske from 4 to 2 and back to 4.


/** PID Controller parameters */
typedef struct {
	/** @brief offset see MM - Master-Gain ---------------------- Fenske
	 *
	 *  bits [31:14] - Reserved
	 *  bit  [13: 0] - Master-Gain (signed)
	 */
	uint32_t gain;
    /** @brief offset see MM - Set-point
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - Set-point (signed)
     */
    uint32_t setpoint;
    /** @brief offset see MM - Proportional gain
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - Proportional gain (signed)
     */
    uint32_t kp;
    /** @brief offset see MM - Integral gain
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - Integral gain (signed)
     */
    uint32_t ki;
    /** @brief offset see MM - Derivative gain
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - Derivative gain (signed)
     */
    uint32_t kd;
    /** @brief offset see MM - Output Limit --------------------------- Fenske
         *
         *  bits [31:14] - Reserved
         *  bit  [13: 0] - Limit (signed)
         */
    uint32_t limit_up; // --------------------------------------------------------- Fenske
    uint32_t limit_low; // --------------------------------------------------------- Fenske
    uint32_t int_limit; // integrator limit
    /** @brief offset see MM - SECOND Integral gain
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - SECOND Integral gain (signed)
     */
    uint32_t kii;
    uint32_t count_ki;
    /** @brief offset see MM - Counter for values that should be integrated
     *
     *  bits [31:14] - Reserved
     *  bit  [13: 0] - Counter KI (signed)
     */
    uint32_t res0;
        /** @brief Offset 0x38 - Reserved */
    uint32_t res1;
        /** @brief Offset 0x3C - Reserved */
} pid_param_t;

// bar graph data ----------------------------------- Fenske
typedef struct {
	uint32_t p;
    uint32_t i;
    uint32_t d;
    uint32_t o;
} meas_ch_t;


/** @brief PID FPGA registry structure.
 *
 * This structure is direct image of physical FPGA memory. When accessing it all
 * reads/writes are performed directly from/to FPGA PID registers.
 */

/* changed by Fenske ---------------------------------------------------------------*/
typedef struct pid_reg_t {
    /** @brief Offset 0x00 - Configuration
     *
     *  bits [31:4]  - Reserved
     *  bit  [   3] -  PID22 automatic integrator reset
     *  bit  [   2] -  PID11 automatic integrator reset
     *  bit  [   1] -  PID22 integrator reset
     *  bit  [   0] -  PID11 integrator reset
     */
    uint32_t configuration;
    /** @brief Offset 0x04 - Reserved */
    uint32_t res0;
    /** @brief Offset 0x08 - Reserved */
    uint32_t res1;
    /** @brief Offset 0x0c - Reserved */
    uint32_t res2;

    /** @brief Offset 0x10 - Two PID Controller parameter blocks
     *  pid[0] ... PID11
     *  pid[1] ... PID22       
     */
    pid_param_t pid[NUM_OF_PIDS];

    meas_ch_t meas[2]; // bar graph ----------------- Fenske meas_ch_t meas[NUM_OF_PIDS];

    uint32_t rot_encoder; // rotary encoder ----------- Fenske

    uint32_t out_1_offset; // output 1 offset ----------- Fenske

    uint32_t out_2_offset; // output 2 offset ----------- Fenske

} pid_reg_t;

/** @} */

/* Description of the following variables or function declarations is in 
 * fpga_pid.c
 */
extern pid_reg_t    *g_pid_reg;

int fpga_pid_init(void);
int fpga_pid_exit(void);

#endif // _FPGA_PID_H_

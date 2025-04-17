#ifndef __DM_DRV_H__
#define __DM_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE   0x000
#define POS_MODE   0x100
#define SPEED_MODE 0x200
#define NUM_MOTORS 8

// Coefficient limits
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

//DM4310 limits
#define P_MAX_4310 12.5f
#define V_MAX_4310 30.0f
#define T_MAX_4310 10.0f

//DM4340 limits
#define P_MAX_4340 12.5f
#define V_MAX_4340 8.0f
#define T_MAX_4340 28.0f

//DM8009 limits
#define P_MAX_8009 12.5f
#define V_MAX_8009 45.0f
#define T_MAX_8009 54.0f

//DM3507 limits - ???

// full parameter list + details - https://github.com/cmjang/DM_Control_Python
typedef enum {
    RID_UV_VALUE=0, //Under-voltage value RW
    RID_KT_VALUE=1, // Torque coefficient RW
    RID_OT_VALUE=2, // Over-temperature RW
    RID_OC_VALUE=3,  // Over-current value RW
    RID_ACC		=4, // Acceleration RW
    RID_DEC		=5, // Deceleration RW
    RID_MAX_SPD	=6, // Maximum Speed RW
    RID_MST_ID	=7, // Feedback ID RW
    RID_ESC_ID	=8, // Receive ID RW
    RID_TIMEOUT	=9, // Timeout Alarm Time RW
    RID_CMODE	=10, // Control Mode RW
    RID_DAMP	=11, // Motor damping coefficient RW
    RID_INERTIA =12, // Motor Inertia RO
    RID_HW_VER	=13, // Reserved RO
    RID_SW_VER	=14, // Software Version RO
    RID_SN		=15, // Reserved RO
    RID_NPP		=16, // Motor Pole Pairs RO
    RID_RS		=17, // Motor Phase Resistance RO
    RID_LS		=18, // Motor Phase Inductance RO
    RID_FLUX	=19, // Motor Flux value RO
    RID_GR		=20, // Gear Reduction Ratio RO
    RID_PMAX	=21, // Position Mapping Max RW
    RID_VMAX	=22, // Speed Mapping Max RW
    RID_TMAX	=23, // Torque Mapping Max RW
    RID_I_BW	=24, // Current Loop Bandwidth RW
    RID_KP_ASR	=25, // Speed Loop Kp RW
    RID_KI_ASR	=26, // Speed Loop Ki RW
    RID_KP_APR	=27, // Position Loop Kp RW
    RID_KI_APR	=28, // Position Loop Ki RW
    RID_OV_VALUE=29, // Over-voltage Value RW
    RID_GREF	=30, // Gear Torque Efficiency RW
    RID_DETA	=31, // Speed Loop Damping Coeff. RW
    RID_V_BW	=32, // Speed Loop Filter Bandwidth RW
    RID_IQ_CL	=33, // Current Loop Gain RW
    RID_VL_CL	=34, // Speed Loop Gain RW
    RID_CAN_BR	=35, // CAN Baud Rate Code RW
    RID_SUB_VER	=36, // Sub-version RO
    RID_U_OFF	=50, // U Phase Offset RO
    RID_V_OFF	=51, // V Phase Offset RO
    RID_K1		=52, // Compensation Factor 1 RO
    RID_K2		=53, // Compensation Factor 2 RO
    RID_M_OFF	=54, // Angle Offset RO
    RID_DIR		=55, // Direction RO
    RID_P_M		=80, // Motor Position RO
    RID_X_OUT	=81 // Output Shaft Position RO
} rid_e;

typedef enum
{
	DM4310 = 0,
	DM4340 = 1,
	DM8009 = 2,
	DM3507 = 3
} MotorType_t;

typedef enum {
    BAUD_125K   = 0,
    BAUD_200K   = 1,
    BAUD_250K   = 2,
    BAUD_500K   = 3,
    BAUD_1M     = 4,
    BAUD_2M     = 5,
    BAUD_2_5M   = 6,
    BAUD_3_2M   = 7,
    BAUD_4M     = 8,
    BAUD_5M     = 9
} CAN_Baudrate_t;

typedef struct
{
  uint16_t slave_id;
	uint16_t master_id;
	MotorType_t type;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;
	
	// motor limits
	float PMAX; 
  float VMAX;		
  float TMAX;
}Joint_Motor_t;

typedef struct 
{
	Joint_Motor_t motors[NUM_MOTORS];
	int mode;
}OpenArm_t;

extern void dm_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void set_zero_position(hcan_t* hcan, uint16_t motor_id);
extern void change_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate);

extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id, uint16_t master_id, uint16_t type);

	
extern float Hex_To_Float(uint32_t *Byte,int num);
extern uint32_t FloatTohex(float HEX);

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode, int motor_type[]);
extern void openarm_enable(OpenArm_t *arm,  hcan_t *hcan);
extern void openarm_disable(OpenArm_t *arm,  hcan_t *hcan);
extern void move_mit_all(OpenArm_t *arm, hcan_t *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]);
#endif /* __DM_DRV_H__ */


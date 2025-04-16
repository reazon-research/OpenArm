#ifndef __DM_DRV_H__
#define __DM_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE   0x000
#define POS_MODE   0x100
#define SPEED_MODE 0x200
#define NUM_MOTORS 8

//DM4310 limits
#define P_MIN_4310 -12.5f
#define P_MAX_4310 12.5f
#define V_MIN_4310 -30.0f
#define V_MAX_4310 30.0f
#define KP_MIN_4310 0.0f
#define KP_MAX_4310 500.0f
#define KD_MIN_4310 0.0f
#define KD_MAX_4310 5.0f
#define T_MIN_4310 -10.0f
#define T_MAX_4310 10.0f

//DM4340 limits
#define P_MIN_4340 -12.5f
#define P_MAX_4340 12.5f
#define V_MIN_4340 -8.0f
#define V_MAX_4340 8.0f
#define KP_MIN_4340 0.0f
#define KP_MAX_4340 500.0f
#define KD_MIN_4340 0.0f
#define KD_MAX_4340 5.0f
#define T_MIN_4340 -28.0f
#define T_MAX_4340 28.0f

//DM8009 limits
#define P_MIN_8009 -12.5f
#define P_MAX_8009 12.5f
#define V_MIN_8009 -45.0f
#define V_MAX_8009 45.0f
#define KP_MIN_8009 0.0f
#define KP_MAX_8009 500.0f
#define KD_MIN_8009 0.0f
#define KD_MAX_8009 5.0f
#define T_MIN_8009 -54.0f
#define T_MAX_8009 54.0f

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
}Joint_Motor_t ;

typedef struct 
{
	Joint_Motor_t motors[NUM_MOTORS];
	int mode;
}OpenArm_t;

extern void dm_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
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


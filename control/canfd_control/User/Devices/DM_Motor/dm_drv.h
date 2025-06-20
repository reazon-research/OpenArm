#ifndef __DM_DRV_H__
#define __DM_DRV_H__
#include "fdcan.h"

// Modes
#define MIT_MODE 0x000

#define NUM_MOTORS 8

// Coefficient limits
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

// DM4310 limits
#define P_MAX_4310 12.5f
#define V_MAX_4310 30.0f
#define T_MAX_4310 10.0f

// DM4340 limits
#define P_MAX_4340 12.5f
#define V_MAX_4340 8.0f
#define T_MAX_4340 28.0f

// DM8009 limits
#define P_MAX_8009 12.5f
#define V_MAX_8009 45.0f
#define T_MAX_8009 54.0f

// DM3507 limits
#define P_MAX_3507 12.566f
#define V_MAX_3507 50.0f
#define T_MAX_3507 5.0f

typedef enum
{
	DM4310 = 0,
	DM4340 = 1,
	DM8009 = 2,
	DM3507 = 3
} MotorType_t;

typedef struct
{
	uint16_t slave_id;
	uint16_t master_id;
	MotorType_t type;
} Joint_Motor_t;

typedef struct
{
	Joint_Motor_t motors[NUM_MOTORS];
	int mode;
} OpenArm_t;

extern void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

extern void mit_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);

extern void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t master_id, MotorType_t type);

extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode, MotorType_t motor_type[]);
extern void openarm_enable(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan);
extern void openarm_disable(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan);
extern void move_mit_all(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]);

#endif /* __DM_DRV_H__ */


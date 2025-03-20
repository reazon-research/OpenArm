#ifndef __OPENARM_CONTROL_H__
#define __OPENARM_CONTROL_H__

#include "dm_drv.h"

#define NUM_MOTORS 7

typedef struct 
{
	Joint_Motor_t motors[NUM_MOTORS];
	int mode;
}OpenArm_t;

extern void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode, int motor_type[]);
extern void openarm_enable(OpenArm_t *arm,  hcan_t *hcan);
extern void openarm_disable(OpenArm_t *arm,  hcan_t *hcan);
extern void move_mit_all(OpenArm_t *arm, hcan_t *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]);
#endif /* __OPENARM_CONTROL_H__ */
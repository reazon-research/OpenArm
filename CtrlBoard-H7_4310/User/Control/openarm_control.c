#include "openarm_control.h"

/**
************************************************************************
* @brief:      	openarm_init: initializes OpenArm motors with provided parameters
* @param[in]:   arm:	pointer to OpenArm struct
* @param[in]:   id[]:	array of slave ids 
* @param[in]:   motor_type[]: array of motor types, refer to MotorType_t
* @param[in]:   mode[]: array of motor control modes
* @retval:     	void
* @details:    	
************************************************************************
**/
void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode[], int motor_type[]){
	for(int i = 0; i < NUM_MOTORS; i++){
		joint_motor_init(&arm->motors[i], id[i], master_id[i], mode[i], motor_type[i]);
	}
}

/**
************************************************************************
* @brief:      	move_mit: moves robot arm according to MIT control params
* @param[in]:   arm:	pointer to OpenArm struct
* @param[in]:   hcan:	TODO
* @param[in]:   position[]:	TODO
* @param[in]:   kp[]: TODO
* @param[in]:   kd[]: TODO
* @param[in]:   tau[]: TODO
* @retval:     	void
* @details:    	
************************************************************************
**/
void move_mit_all(OpenArm_t *arm, hcan_t *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]){
	for(int i = 0; i < NUM_MOTORS; i++){
		mit_ctrl(hcan, arm->motors[i].para.slave_id, position[i], velocity[i], kp[i], kd[i], torque[i]);
	}
}
	

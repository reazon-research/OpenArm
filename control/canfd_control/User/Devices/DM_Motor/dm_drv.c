#include "dm_drv.h"
#include "can_bsp.h"

#include "EventRecorder.h"

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t master_id, MotorType_t type)
{
	motor->slave_id = id;
	motor->master_id= master_id;
	motor->type = type;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: enables motor
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to disable
* @param[in]:   mode_id: the mode id defined in dm_drv.h
* @retval:     	void
************************************************************************
**/
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: disables motor
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to disable
* @param[in]:   mode_id: the mode id defined in dm_drv.h
* @retval:     	void
************************************************************************
**/
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}


/**
************************************************************************
* @brief:      	mit_ctrl: moves one motor using MIT control based on given parameters
* @param[in]:   arm:	pointer to OpenArm struct
* @param[in]:   hcan:	handler for FDCAN bus
* @param[in]:   motor_id:	slave_id of motor in CAN bus
* @param[in]:   pos: desired position
* @param[in]:   vel: desired velocity
* @param[in]:   kp: position proportional gain
* @param[in]:   kd: position differential gain
* @param[in]:   torq: feedforward torque
* @retval:     	void
************************************************************************
**/
void mit_ctrl(OpenArm_t *arm, FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	Joint_Motor_t *motor = &(arm->motors[motor_id-1]);
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;
        float p_max; // Max position range [rad] (RW)
        float v_max; // Max speed range [rad/s] (RW)
        float t_max; // Max torque range [Nm] (RW)

	switch(motor->type)
	{
	case DM4310:
		p_max = P_MAX_4310;
		v_max = V_MAX_4310;
		t_max = T_MAX_4310;
		break;
	case DM4340:
		p_max = P_MAX_4340;
		v_max = V_MAX_4340;
		t_max = T_MAX_4340;
		break;
	case DM8009:
		p_max = P_MAX_8009;
		v_max = V_MAX_8009;
		t_max = T_MAX_8009;
		break;
	case DM3507:
		p_max = P_MAX_3507;
		v_max = V_MAX_3507;
		t_max = T_MAX_3507;
		break;
	}

	pos_tmp = float_to_uint(pos, -p_max, p_max, 16);
	vel_tmp = float_to_uint(vel, -v_max, v_max, 12);
	tor_tmp = float_to_uint(torq, -t_max, t_max, 12);
	kp_tmp  = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd, KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;

	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	openarm_init: initializes OpenArm motors with provided parameters
* @param[in]:   arm:	pointer to OpenArm struct
* @param[in]:   id[]:	array of slave ids 
* @param[in]:   motor_type[]: array of motor types, refer to MotorType_t
* @param[in]:   mode[]: array of motor control modes
* @retval:     	void
************************************************************************
**/
void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode, MorterType_t motor_type[]){
	arm->mode = mode;
	for(int i = 0; i < NUM_MOTORS; i++){
		joint_motor_init(&arm->motors[i], id[i], master_id[i], motor_type[i]);
	}
}

void openarm_enable(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan){
	for(int i = 0; i < NUM_MOTORS; i++){
		enable_motor_mode(hcan, arm->motors[i].slave_id, arm->mode);
	}
}

void openarm_disable(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan){
	for(int i = 0; i < NUM_MOTORS; i++){
		disable_motor_mode(hcan, arm->motors[i].slave_id, arm->mode);
	}
}

/**
************************************************************************
* @brief:      	move_mit: moves OpenArm according to MIT control params
* @param[in]:   arm:	pointer to OpenArm struct
* @param[in]:   hcan:	handler for CANFD interface
* @param[in]:   position[]:	array of desired position values
* @param[in]:   position[]:	array of desired velocity values
* @param[in]:   kp[]: array of position proportional gain
* @param[in]:   kd[]: array of position differential gain
* @param[in]:   tau[]: array of torque
* @retval:     	void
************************************************************************
**/
void move_mit_all(OpenArm_t *arm, FDCAN_HandleTypeDef *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]){
	for(int i = 0; i < NUM_MOTORS; i++){
		mit_ctrl(arm, hcan, arm->motors[i].slave_id, position[i], velocity[i], kp[i], kd[i], torque[i]);
		while (HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0) {}
	}
}



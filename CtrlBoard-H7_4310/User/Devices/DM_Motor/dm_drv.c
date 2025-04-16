#include "dm_drv.h"

#include "fdcan.h"
#include "arm_math.h"
#include "EventRecorder.h"
#include "stdio.h"

float Hex_To_Float(uint32_t *Byte,int num)
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)
{
	return *( uint32_t *)&HEX;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor,uint16_t id, uint16_t master_id, uint16_t type)
{
	motor->slave_id=id;
	motor->master_id=master_id;
	motor->type = type;
}

// feedback message callback function
void dm_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	
	if(data_len==FDCAN_DLC_BYTES_8)
	{
	}	  
		//motor->slave_id = (rx_data[0])&0x0F;
	  motor->state = (rx_data[0])>>4;
	  motor->p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
		switch(motor->type)
		{
			case DM4310:
				motor->pos = uint_to_float(motor->p_int, P_MIN_4310, P_MAX_4310, 16); // 
				motor->vel = uint_to_float(motor->v_int, V_MIN_4310, V_MAX_4310, 12); // 
				motor->tor = uint_to_float(motor->t_int, T_MIN_4310, T_MAX_4310, 12);  // 
			case DM4340:
				motor->pos = uint_to_float(motor->p_int, P_MIN_4340, P_MAX_4340, 16); // 
				motor->vel = uint_to_float(motor->v_int, V_MIN_4340, V_MAX_4340, 12); // 
				motor->tor = uint_to_float(motor->t_int, T_MIN_4340, T_MAX_4340, 12);  // 		
			case DM8009:
				motor->pos = uint_to_float(motor->p_int, P_MIN_8009, P_MAX_8009, 16); // 
				motor->vel = uint_to_float(motor->v_int, V_MIN_8009, V_MAX_8009, 12); // 
				motor->tor = uint_to_float(motor->t_int, T_MIN_8009, T_MAX_8009, 12);  // 		
			case DM3507:
			default: break;
		}
	  
		//EventRecord2(0x01, rx_data[0]&0x0F, motor->vel);
		//printf("ID: %d Velocity: %0.2f\r\n", (rx_data[0])&0x0F, motor->vel);
	  motor->Tmos = (float)(rx_data[6]);
	  motor->Tcoil = (float)(rx_data[7]);

}

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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
* @brief:      	disable_motor_mode: disables mode on desired motor
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to disable
* @param[in]:   mode_id: the mode id defined in dm_drv.h
* @retval:     	void
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
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
* @brief:      	change_baudrate: changes baudrate to 5Mbps
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to change baudrate for
* @retval:     	void
************************************************************************
**/
void change_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate){
	uint8_t command1[4] = {0x00, 0x00, 0x33, 0x23};
	uint8_t command2[8] = {0x00, 0x00, 0x55, 0x23, baudrate, 0x00, 0x00, 0x00};
	uint8_t command3[4] = {0x00, 0x00, 0xAA, 0x23};
	// Array of pointers to the commands
	uint8_t* command_list[3] = {command1, command2, command3};
	
	for (int i = 0; i < 3; ++i) {
				int dlc = (i == 1) ? 8 : 4;  // command 1 (index 1) is 8 bytes, others are 4
        command_list[i][0] = motor_id;  // replace first byte with motor ID
        canx_send_data(hcan, 0x7FF, command_list[i], dlc); // assuming standard CAN ID
        printf("Sent to motor 0x%02X: [", motor_id);
        for (int j = 0; j < dlc; ++j) printf(" %02X", command_list[i][j]);
        printf(" ]\n");

        HAL_Delay(10);
	}
}


/**
************************************************************************
* @brief:      	mit_ctrl: moves one motor using MIT control based on given parameters
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
extern OpenArm_t arm;
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;
	
	switch(arm.motors[motor_id-1].type)
	{
		case DM4310:
			pos_tmp = float_to_uint(pos,  P_MIN_4310, P_MAX_4310,  16);
			vel_tmp = float_to_uint(vel,  V_MIN_4310,  V_MAX_4310,  12);
			kp_tmp  = float_to_uint(kp,   KP_MIN_4310, KP_MAX_4310, 12);
			kd_tmp  = float_to_uint(kd,   KD_MIN_4310, KD_MAX_4310, 12);
			tor_tmp = float_to_uint(torq, T_MIN_4310,  T_MAX_4310,  12);
			break;
		case DM4340:
			pos_tmp = float_to_uint(pos,  P_MIN_4340,  P_MAX_4340,  16);
			vel_tmp = float_to_uint(vel,  V_MIN_4340,  V_MAX_4340,  12);
			kp_tmp  = float_to_uint(kp,   KP_MIN_4340, KP_MAX_4340, 12);
			kd_tmp  = float_to_uint(kd,   KD_MIN_4340, KD_MAX_4340, 12);
			tor_tmp = float_to_uint(torq, T_MIN_4340,  T_MAX_4340,  12);	
			break;
		case DM8009:
			pos_tmp = float_to_uint(pos,  P_MIN_8009,  P_MAX_8009,  16);
			vel_tmp = float_to_uint(vel,  V_MIN_8009,  V_MAX_8009,  12);
			kp_tmp  = float_to_uint(kp,   KP_MIN_8009, KP_MAX_8009, 12);
			kd_tmp  = float_to_uint(kd,   KD_MIN_8009, KD_MAX_8009, 12);
			tor_tmp = float_to_uint(torq, T_MIN_8009,  T_MAX_8009,  12);	
			break;
		case DM3507:
			break;
		default: printf("Motor type is invalid\n"); return;
	}

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
* @brief:      	pos_speed_ctrl: position cascade mode that uses three-loop series control
* @param[in]:   hcan: handler for CANFD bus
* @param[in]:   motor_id:	slave id of motor in CAN bus
* @param[in]:   vel: desired velocity
* @retval:     	void
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: runs motor at set speed
* @param[in]:   hcan: handler for CANFD bus
* @param[in]:   motor_id: slave id of motor in CAN bus
* @param[in]:   vel: desired velocity
* @retval:     	void
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
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
void openarm_init(OpenArm_t *arm, int id[], int master_id[], int mode, int motor_type[]){
	arm->mode = mode;
	for(int i = 0; i < NUM_MOTORS; i++){
		joint_motor_init(&arm->motors[i], id[i], master_id[i], motor_type[i]);
	}
}

void openarm_enable(OpenArm_t *arm,  hcan_t *hcan){
	for(int i = 0; i < NUM_MOTORS; i++){
		enable_motor_mode(hcan, arm->motors[i].slave_id, arm->mode);
	}
}

void openarm_disable(OpenArm_t *arm,  hcan_t *hcan){
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
void move_mit_all(OpenArm_t *arm, hcan_t *hcan, float position[], float velocity[], float kp[], float kd[], float torque[]){
	for(int i = 0; i < NUM_MOTORS; i++){
		mit_ctrl(hcan, arm->motors[i].slave_id, position[i], velocity[i], kp[i], kd[i], torque[i]);
		while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {}
	}
}



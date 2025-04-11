#include "dm_drv.h"

#include "fdcan.h"
#include "arm_math.h"
#include "EventRecorder.h"
#include "stdio.h"

float Hex_To_Float(uint32_t *Byte,int num)//ʮ�����Ƶ�������
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)//��������ʮ������ת��
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
		if(motor->type == DM4310){
			motor->pos = uint_to_float(motor->p_int, P_MIN_4310, P_MAX_4310, 16); // 
			motor->vel = uint_to_float(motor->v_int, V_MIN_4310, V_MAX_4310, 12); // 
			motor->tor = uint_to_float(motor->t_int, T_MIN_4310, T_MAX_4310, 12);  // 
		}
		else if(motor->type == DM4340){
			motor->pos = uint_to_float(motor->p_int, P_MIN_4340, P_MAX_4340, 16); // 
			motor->vel = uint_to_float(motor->v_int, V_MIN_4340, V_MAX_4340, 12); // 
			motor->tor = uint_to_float(motor->t_int, T_MIN_4340, T_MAX_4340, 12);  // 		
		}
	  
		EventRecord2(0x01, rx_data[0]&0x0F, motor->vel);
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
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
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
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
extern OpenArm_t arm;
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;
	if(arm.motors[motor_id-1].type == DM4310){
		pos_tmp = float_to_uint(pos,  P_MIN_4310, P_MAX_4310,  16);
		vel_tmp = float_to_uint(vel,  V_MIN_4310,  V_MAX_4310,  12);
		kp_tmp  = float_to_uint(kp,   KP_MIN_4310, KP_MAX_4310, 12);
		kd_tmp  = float_to_uint(kd,   KD_MIN_4310, KD_MAX_4310, 12);
		tor_tmp = float_to_uint(torq, T_MIN_4310,  T_MAX_4310,  12);
	}
	else if(arm.motors[motor_id-1].type == DM4340){
		pos_tmp = float_to_uint(pos,  P_MIN_4340,  P_MAX_4340,  16);
		vel_tmp = float_to_uint(vel,  V_MIN_4340,  V_MAX_4340,  12);
		kp_tmp  = float_to_uint(kp,   KP_MIN_4340, KP_MAX_4340, 12);
		kd_tmp  = float_to_uint(kd,   KD_MIN_4340, KD_MAX_4340, 12);
		tor_tmp = float_to_uint(torq, T_MIN_4340,  T_MAX_4340,  12);	
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
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
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
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
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
* @details:    	
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
		mit_ctrl(hcan, arm->motors[i].slave_id, position[i], velocity[i], kp[i], kd[i], torque[i]);
		while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {}
	}
}



#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

extern void set_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate, uint8_t mode);
extern void write_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate);

#endif


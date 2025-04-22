#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#include "main.h"

#define CAN_CLASS   0
#define CAN_FD_BRS  1

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

typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

extern void set_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate, uint8_t mode);
extern void write_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate);

#endif


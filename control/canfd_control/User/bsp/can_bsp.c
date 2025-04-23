#include "can_bsp.h"
#include "fdcan.h"
#include "dm_drv.h"
#include "string.h"
#include "stdio.h"
#include "EventRecorder.h"


FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0x00000000;
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* Configure and enable Tx Delay Compensation : TdcOffset = DataTimeSeg1*DataPrescaler */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataTimeSeg1*hfdcan1.Init.DataPrescaler, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
 
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
	
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1,
    FDCAN_ACCEPT_IN_RX_FIFO1,
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* Configure and enable Tx Delay Compensation : TdcOffset = DataTimeSeg1*DataPrescaler */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, hfdcan2.Init.DataTimeSeg1*hfdcan2.Init.DataPrescaler, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);
	
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len<<16;     
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }
											
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat =  FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;
	
   
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
       Error_Handler();      
  }
	 return 0;
}


extern OpenArm_t arm;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
			//another example of how to use the event recorder
			EventRecord2(0x01, RxHeader1.Identifier, HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0));
			uint16_t motor_id = -1;  // Initialize motor_id as invalid
			if (RxHeader1.Identifier != 0)
			{
					motor_id = RxHeader1.Identifier - 0x11; // Assuming motor IDs start from 0x11
			}
			else
			{
					motor_id = (g_Can1RxData[0]) & 0x0F;  // Mask to extract lower 4 bits
			}
			if (motor_id >= 0 && motor_id < NUM_MOTORS)
			{
					dm_fbdata(&arm.motors[motor_id], g_Can1RxData, RxHeader1.DataLength);
			}
			else
			{
					printf("Invalid motor ID: %d (CAN ID: 0x%X)\n\r", motor_id, RxHeader1.Identifier);
			}
		}			
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN2)
    {
      /* Retrieve Rx messages from RX FIFO1 */
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			
			//another example of how to use the event recorder
			EventRecord2(0x01, RxHeader2.Identifier, HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1));
			uint16_t motor_id = -1;  // Initialize motor_id as invalid
			if (RxHeader2.Identifier != 0)
			{
					motor_id = RxHeader2.Identifier - 0x11; // Assuming motor IDs start from 0x11
			}
			else
			{
					motor_id = (g_Can2RxData[0]) & 0x0F;  // Mask to extract lower 4 bits
			}
			if (motor_id >= 0 && motor_id < NUM_MOTORS)
			{
					dm_fbdata(&arm.motors[motor_id], g_Can2RxData, RxHeader2.DataLength);
			}
			else
			{
					printf("Invalid motor ID: %d (CAN ID: 0x%X)\n\r", motor_id, RxHeader2.Identifier);
			}
    }
  }
}

/**
************************************************************************
* @brief:      	change_baudrate: sets baudrate to desired baudrate until power cycle
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to change baudrate for
* @retval:     	void
* @details:    	use inside FDCAN_Config before HAL_FDCAN_Start
************************************************************************
**/
void set_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate, uint8_t mode){
	if(mode == CAN_CLASS)
	{
		disable_motor_mode(hcan, motor_id, MIT_MODE);
		read_motor_data(hcan, motor_id, RID_CAN_BR);
		HAL_Delay(100);
		change_motor_data(hcan, motor_id, RID_CAN_BR, baudrate,0,0,0);
		HAL_Delay(100);
		enable_motor_mode(hcan, motor_id, MIT_MODE);
		
		hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
		hcan->Init.DataPrescaler = 4;
		hcan->Init.DataTimeSeg1 = 19;
		hcan->Init.DataTimeSeg2 = 5;
		hcan->Init.DataSyncJumpWidth = 1;
	}
	else if (mode == CAN_FD_BRS)
	{
		disable_motor_mode(hcan, motor_id, MIT_MODE);
		read_motor_data(hcan, motor_id, RID_CAN_BR);
		HAL_Delay(100);
		change_motor_data(hcan, motor_id, RID_CAN_BR, baudrate,0,0,0);
		HAL_Delay(100);
		enable_motor_mode(hcan, motor_id, MIT_MODE);
		
		hcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
		hcan->Init.DataPrescaler = 1;
		hcan->Init.DataTimeSeg1 = 16;
		hcan->Init.DataTimeSeg2 = 3;
		hcan->Init.DataSyncJumpWidth = 2;
	}
}

/**
************************************************************************
* @brief:      	write_baudrate: changes baudrate to desired baudrate permanently
* @param[in]:   hcan: handler for CANFD interface
* @param[in]:   motor_id: slave id of motor to change baudrate for
* @retval:     	void
************************************************************************
**/
void write_baudrate(hcan_t* hcan, uint16_t motor_id, uint8_t baudrate){
	disable_motor_mode(hcan, motor_id, MIT_MODE);
	read_motor_data(hcan, motor_id, RID_CAN_BR);
	HAL_Delay(100);
	change_motor_data(hcan, motor_id, RID_CAN_BR, baudrate,0,0,0);
	HAL_Delay(100);
	write_motor_data(hcan, motor_id);
	HAL_Delay(100);
	enable_motor_mode(hcan, motor_id, MIT_MODE);
}




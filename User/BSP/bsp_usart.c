#include "bsp_usart.h"

#include "usart.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "fifo.h"
//涓插彛5锛屾帴鏀堕仴鎺ф暟鎹?


//涓插彛1锛屾帴鏀惰鍒ょ郴缁熸暟鎹?
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
fifo_s_t referee_fifo;
uint8_t referee_fifo_ready = 0;
static uint8_t usart1_buf_index = 0;


uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;

//涓插彛7 鏉块棿閫氳
//static uint8_t gimbal_data[USART_RX_BUF_LENGHT]={0};
uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//璁剧疆缂撳啿鍖?
uint8_t data_send_from_pc[ USART_RX_BUF_LENGHT] = {0};
uint8_t data_send_from_chassis[ USART_RX_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//璇锋眰閲嶆柊鍙戦€佹暟鎹?

//涓插彛10 浜戝彴鏉夸笌鍥句紶閫氫俊, 鐢ㄤ簬缁樺埗UI
uint8_t usart10_buf[ USART_RX_BUF_LENGHT ];//璁剧疆缂撳啿鍖?

//涓插彛1鍙戦€佸嚱鏁?
void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

//涓插彛1鍙戦€佸嚱鏁?
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

//涓插彛1鍙戦€佸嚱鏁?
void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart1, pData, Size, 50);
}

//涓插彛7鍙戦€佸嚱鏁?
void USART7_Transmit(uint8_t *pData, uint16_t Size)
{
  if (huart7.gState == HAL_UART_STATE_READY)
  {
    HAL_UART_Transmit_DMA(&huart7, pData, Size);
  }
}

//涓插彛8鍙戦€佸嚱鏁?
void USART8_Transmit(uint8_t *pData, uint16_t Size)
{
  if (huart8.gState == HAL_UART_STATE_READY)
  {
    HAL_UART_Transmit_DMA(&huart8, pData, Size);
  }
}

//涓插彛10鍙戦€佸嚱鏁?//void USART10_Transmit(uint8_t *pData, uint16_t Size)
//{
//	HAL_UART_Transmit(&huart10, pData, Size, 50);
//}

////涓插彛10涓柇鍙戦€佸嚱鏁?
//void USART10_Transmit_IT(uint8_t *pData, uint16_t Size)
//{
//	HAL_UART_Transmit_IT(&huart10, pData, Size);
//}

gimbal_data_t gimbal;



float yaw_motor_relative_angle, GIMBAL_INS_yaw;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7) {
		// 澶勭悊鎺ユ敹鍒扮殑鏁版嵁
		// 渚嬪锛屽皢鎺ユ敹鍒扮殑鏁版嵁瀛樺叆缂撳啿鍖烘垨瑙﹀彂鏌愮浜嬩欢

		// 缁х画鎺ユ敹涓嬩竴涓暟鎹潡
	}
	else if (huart->Instance == USART10) {

	}
}
/*绌洪棽涓柇鍥炶皟*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		if((referee_fifo_ready != 0) && (Size <= USART_RX_BUF_LENGHT))
		{
			fifo_s_puts(&referee_fifo, (char *)usart1_buf[usart1_buf_index], Size);
		}
		usart1_buf_index ^= 1U;
		memset(usart1_buf[usart1_buf_index], 0, USART_RX_BUF_LENGHT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[usart1_buf_index], USART_RX_BUF_LENGHT);
	}
	if(huart->Instance == UART5)
	{
		if (Size <= RC_FRAME_LENGTH)
		{
			sbus_to_rc(remote_buff, &rc_ctrl, &remoter);
		}
		else if(Size > RC_FRAME_LENGTH) // 閿欒澶勭悊
		{
			memset(remote_buff, 0, SBUS_RX_BUF_NUM);
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM);
	}
	if(huart->Instance == UART7)
	{
		hwt101_rx_parse(usart7_buf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7_buf, USART_RX_BUF_LENGHT);
	}

	if(huart->Instance == USART10)
	{
		hwt906_rx_parse(usart10_buf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART_RX_BUF_LENGHT);
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1){
		memset(usart1_buf, 0,  USART_RX_BUF_LENGHT * 2);
		usart1_buf_index = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[usart1_buf_index], USART_RX_BUF_LENGHT);
	}
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM); // 鎺ユ敹鍙戠敓閿欒鍚庨噸鍚?
		memset(remote_buff, 0, SBUS_RX_BUF_NUM);							   // 娓呴櫎鎺ユ敹缂撳瓨
	}
	if(huart->Instance == UART7)
	{
		memset(usart7_buf, 0, USART_RX_BUF_LENGHT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7_buf, USART_RX_BUF_LENGHT);
	}
	if(huart->Instance == USART10)
	{
		memset(usart10_buf, 0, USART_RX_BUF_LENGHT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART_RX_BUF_LENGHT);

	}
}

/* 杞彂 TX 瀹屾垚鍥炶皟 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* removed: uart_dma_tx_cplt_handler */
}


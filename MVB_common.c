
#include <MVB_common.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart8;

const UAcpEndian g_uAcpEndian  = { 0x44, 0x33, 0x22, 0x11 };

static uint16_t CRC16 (uint8_t* pu8Data, uint32_t u32Len);
void MX_USART1_UART_Init(void);
void MX_UART8_Init(void);

uint8_t acpCommPutSerial (uint8_t* pu8TxData, uint16_t u16Length);
EAcpSessResult acpCommRun (SAcpDgram *psReqDgram);


/* --------------------------------------------------------
 Input    :  pu8Data  -  pointer to data
             u32Len   -  data length
 Output   :  -
 Return   :  Counted CRC
 Descr    :  Computing CRC
 ------------------------------------------------------- */
static uint16_t CRC16 (uint8_t* pu8Data, uint32_t u32Len)
{
  uint16_t u16Crc = 0xFFFF;  /* CRC */

  for ( ; u32Len != 0; u32Len--)
  {
    u16Crc  = (uint8_t)(u16Crc >> 8) | (u16Crc << 8);
    u16Crc ^= *pu8Data++;
    u16Crc ^= ((u16Crc & 0xff) >> 4);
    u16Crc ^= (u16Crc << 8) << 4;
    u16Crc ^= ((u16Crc & 0xff) << 4) << 1;
  }

  return u16Crc;
}  /* CRC16 */



/* --------------------------------------------------------
 Input    :  pu8TxData  -  pointer to data
             u16Length  -  data length
 Output   :  -
 Return   :  -
 Descr    :  Put data to the serial port
 ------------------------------------------------------- */

uint8_t acpCommPutSerial (uint8_t* pu8TxData, uint16_t u16Length)
{
	static uint8_t mvb_count=0;

	static uint8_t mvb_RxBuffer[200];
	static uint8_t mvb_TxBuffer [1024]={0};  /* transfer buffer */

	static uint8_t a=0;
	static uint8_t b=0;

/*	static uint8_t mvb_passive[]="mvbpassive\n";
	static uint8_t port_config[]="portconfigok\n";
	static uint8_t mvb_regular[]="mvbregular\n";
	static uint8_t port_data_write[]="portdatawrite\n";
	static uint8_t port_enable[]="portenable\n";
	static uint8_t port_read_config[]="portreadconfig\n";*/

	uint16_t u16TxBufLen = 0;


	/* Fill Tx buffer */
	mvb_TxBuffer[u16TxBufLen++] = DLE;
	mvb_TxBuffer[u16TxBufLen++] = STX;

	for( ; u16Length != 0; u16Length--, pu8TxData++)
	{
		if (*pu8TxData == DLE)
		{
			/* double DLE character */
			mvb_TxBuffer[u16TxBufLen++] = DLE;
		}
		mvb_TxBuffer[u16TxBufLen++] = *pu8TxData;
	}

	mvb_TxBuffer[u16TxBufLen++] = DLE;
	mvb_TxBuffer[u16TxBufLen++] = ETX;

	if(timer > 3 && timer < 8 )
	{
		a = 1;
		b = timer;
		HAL_UART_Receive_IT(&huart8, (uint8_t *)mvb_RxBuffer, 100);

		return uart_rx_ok;
	}



/* TX */
	else if(a==1  && timer > 13)
	{
		a = 2;
		HAL_UART_Transmit(&huart8, (uint8_t *)mvb_TxBuffer ,u16TxBufLen,100);

		return uart_tx_ok;
	}



/* RX */
	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'U'&& mvb_count == 8 && a ==2)
	{
		a = 0;
		mvb_count=5;

		return port_write_disable;
	}


	else if(mvb_RxBuffer[6]==0     && mvb_RxBuffer[5] == 'r'&& mvb_count == 7 && a ==2)
	{
		a = 0;
		mvb_count++;
		HAL_UART_Transmit(&huart1, &mvb_RxBuffer[22], 32,100);

		return mvb_read;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'E'&& mvb_count == 6 && a ==2)
	{
		a = 0;
		mvb_count++;

		return port_write_enable;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'W' && mvb_count == 5 && a ==2 )
	{
		a = 0;
		mvb_count++;

		return port_datawrite;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'r' && mvb_count == 4 && a ==2)
	{
		a = 0;
		mvb_count++;

		return mvb_read;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'E'&& mvb_count == 3  && a ==2)
	{
		a = 0;
		mvb_count++;

		return port_write_enable;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'C' && mvb_count == 2 && a ==2 )
	{
		a = 0;
		mvb_count++;

		return mvb_regular_mode;
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'P' && mvb_count == 1 && a ==2 )
	{
		a = 0;
		mvb_count++;

		return port_config_set_ok;          /* port config ok */
	}


	else if(mvb_RxBuffer[6]==0 && mvb_RxBuffer[5] == 'C' && mvb_count == 0 && a ==2 )
	{
		a = 0;
		mvb_count++;

		return acp_sess_res_ok;
	}

	return nothing_happend;
}


/* --------------------------------------------------------
 Input    :  psReqDgram  -  pointer to request datagram
             psRspDgram  -  pointer to response datagram
 Output   :  -
 Return   :  acp_sess_res_ok  -  success
             other            -  failure
 ------------------------------------------------------- */
EAcpSessResult acpCommRun (SAcpDgram *psReqDgram)
{static uint8_t   s_u8SessSeqNr = 0;     /* global sequential number */

  uint8_t*        	   pu8Data;            /* ptr to request or response data */
  uint16_t             u16Crc;  /* length, CRC */
  uint16_t             u16DataLen;         /* length of data */
  EAcpSessResult  	   eAcpRes;            /* ACP result code */

  /* check length */
  if (psReqDgram->u16DataLen > ACP_DATA_LEN)
    return acp_loc_err_in_dta_siz;

  /* Request data */
  psReqDgram->u8ReqType   = acp_rqtype_req;     /* mvb configure write */
  psReqDgram->u8SessSeqNr = ++s_u8SessSeqNr;
  pu8Data = (uint8_t*)psReqDgram;
  /* count the data length */
  u16DataLen = psReqDgram->u16DataLen + ACP_HDR_CRC_LEN;
  /* code the request data length */
  ACP_PUT_U16 (pu8Data + ACP_FRM_OFFS_LEN - 2, u16DataLen);
  /* Request CRC */
  u16Crc = CRC16 (pu8Data, u16DataLen - 2);
  ACP_PUT_U16 (pu8Data + u16DataLen - 2, u16Crc);

  /* Send Request */
  eAcpRes =acpCommPutSerial (pu8Data, u16DataLen);

  return eAcpRes;
}



void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}


void MX_UART8_Init(void)
{
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_MultiProcessor_Init(&huart8, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
}



/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    pUARTQUEUE pQueue;
    pQueue = (huart->Instance == huart8? &mvbQueue:&MonitorQueue);    // usart2 면 WifiQueue를 아니면 MonitorQueue를 대상으로
    pQueue->head++;                                                     //ⓐ head를 1증가시키고
    if (pQueue->head == 1024) pQueue->head = 0;          //ⓑ head가 Buffer의 끝이면
    pQueue->data++;                                                     //ⓒ data 값을 1증가
    if (pQueue->data == 1024)                            //queue가 full이면 하나 처리하고
        GetDataFromUartQueue(huart);
    HAL_UART_Receive_IT(huart, pQueue->Buffer + pQueue->head, 1);       //ⓓ 데이터 수신 인터럽트 설정
}*/

/*


void GetDataFromUartQueue(UART_HandleTypeDef *huart)
{
    UART_HandleTypeDef *dst = (huart->Instance == UART8 ? &huart8:&huart8);       //USART2이면 출력 대상은 USART3, 아니면 USART2
    pUARTQUEUE pQueue = (huart->Instance == UART8 ? &mvbQueue:&MonitorQueue);     //USART2이면 WifiQueue의 데이터를, 아니면 MonitorQueue의 데이터를
    HAL_UART_Transmit(dst, pQueue->Buffer + pQueue->tail, 1, 3000);                 //한 바이트 전송
    pQueue->tail++;                                                                 //tail 1증가
    if (pQueue->tail ==1024) pQueue->tail = 0;                      //Buffer의 끝이면 0을
    pQueue->data--;                                                                 //data 1감소
    HAL_Delay(1);                                                                   //시간 지연

*/


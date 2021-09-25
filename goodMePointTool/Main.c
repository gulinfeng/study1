/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2019/10/15
* Description        : Main program body.
*******************************************************************************/ 

/*
 *@Note
 GPIO例程：
 PA0推挽输出。
 
*/

#include "debug.h"

#include "oled.h"
#include "bmp.h"
/* Global Variable */ 
u16 TxBuf[1024];   
uint32_t pointNum = 0;

u8 USART2_SendBuff[256];   //发送缓存
u8 USART2_RecieveBuff[256]; //接收数组

__IO uint8_t   USART2_DMAbp = 0;
__IO uint8_t   USART2_DMAep = 0;
uint8_t USART2_commandlong= 0;
uint8_t  USART2_command[256];      //命令缓冲区

uint8_t USART2_commandlong_old = 0;

/* Global define */
#define TxSize1   (size(TxBuffer1))
#define TxSize2   (size(TxBuffer2))
#define size(a)   (sizeof(a) / sizeof(*(a)))

/* Global typedef */
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* Global Variable */
u8 TxBuffer1[] = "*Buffer1 Send from USART2 to USART3 using Interrupt!";     /* Send by UART2 */
u8 TxBuffer2[] = "#Buffer2 Send from USART3 to USART2 using Interrupt!";     /* Send by UART3 */
u8 RxBuffer1[TxSize1]={0};                                                   /* USART2 Using */
u8 RxBuffer2[TxSize2]={0};                                                   /* USART3 Using  */

u8 TxCnt1 = 0, RxCnt1 = 0;
u8 TxCnt2 = 0, RxCnt2 = 0;

u8 Rxfinish1=0,Rxfinish2=0;

TestStatus TransferStatus1 = FAILED;
TestStatus TransferStatus2 = FAILED;

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
* Function Name  : Buffercmp
* Description    : Compares two buffers
* Input          : Buf1,Buf2:buffers to be compared
*                  BufferLength: buffer's length
* Return         : PASSED: Buf1 identical to Buf2
*                  FAILED: Buf1 differs from Buf2
*******************************************************************************/
TestStatus Buffercmp(uint8_t* Buf1, uint8_t* Buf2, uint16_t BufLength)
{
  while(BufLength--)
  {
    if(*Buf1 != *Buf2)
    {
      return FAILED;
    }
    Buf1++;
    Buf2++;
  }
  return PASSED;
}


/*******************************************************************************
* Function Name  : USARTx_CFG
* Description    : Initializes the USART2 & USART3 peripheral.
* Input          : None
* Return         : None
*******************************************************************************/
void USARTx_CFG(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB , ENABLE);

  /* USART2 TX-->A.2   RX-->A.3 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* USART3 TX-->B.10  RX-->B.11 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

uint16_t processPointDeviceCmd(unsigned char buff[], uint16_t buffLength){
    /*unsigned char checkSum = 0, i = 0*/;
    unsigned char lowByte = 0, highByte = 0;
    uint16_t pointCodeValRx = 0;

    if(buffLength != 8){
        return 0;
    }

    if( buff[0] != 0xaa || buff[1] != 0x55){
        return 0;
    }

    if(buff[2] == 0x04 && buff[3] == 0x06 && buff[4] == 0x02){
        /*  心跳包 */
        return 0;
    }

    /*if(buffLength != 13){
        return 0;
    }

    if( buff[0] != 0xFF || buff[1] != 0xFF || buff[2] != 0xFF || buff[3] != 0xAA || buff[4] != 0xBB){
        return 0;
    }

    for(i=0; i<12; i++){
        checkSum += buff[i];
    }*/

    //if(checkSum != buff[12]){
    //  return;
    //}

    /*highByte = buff[10];
    lowByte = buff[11];*/
    highByte = buff[5];
    lowByte = buff[4];
    pointCodeValRx = (((uint16_t)highByte)*256) + lowByte;

    return pointCodeValRx;
}

/*
 *      GND --- GND

　　　　VCC --- VCC

　　　　D0(SCL) --- PB3

　　　　D1(SDA) --- PB4

　　　　RES(RST) --- PB5

　　　　DC(D/C) --- PB6

　　　　CS --- 悬空



    A3接点读码模块  TX glf
 * */

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
    char titleStr[30] = "     GoodMe     ";
    char title2Str[30] ="  TeaCode Tool  ";
    char contStr[15] =  "   code:";
    u16 i, glf1234;
    uint16_t ret = 0, showTimeCnt = 0;
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	
	USARTx_CFG();

	GPIO_OLED_InitConfig();
	OLED_Clear();
	//OLED_DrawBMP(0,0,126,8,BMP);

	OLED_ShowString(0, 0, titleStr);
	OLED_ShowString(0, 3, title2Str);
	OLED_ShowString(0, 6, contStr);

	Delay_Ms(500);

	while(1){

	    if(USART2_commandlong > 0){
	        if(USART2_commandlong_old == USART2_commandlong){

	            ret = processPointDeviceCmd(USART2_RecieveBuff, USART2_commandlong);
	            if(ret > 0){
	                OLED_Clear();
                    sprintf(contStr, "   Code:%d", ret);

                    OLED_ShowString(0, 0, titleStr);
                    OLED_ShowString(0, 3, title2Str);
                    OLED_ShowString(0, 6, contStr);

	            }else{
	                OLED_Clear();
                    sprintf(contStr, "   Code:err");

                    OLED_ShowString(0, 0, titleStr);
                    OLED_ShowString(0, 3, title2Str);
                    OLED_ShowString(0, 6, contStr);
	            }

                for(i=0;i<255;i++){
                    USART2_command[i] = 0;
                }
                USART2_commandlong = 0;
                USART2_commandlong_old = 0;

                showTimeCnt = 3000;

	        }else{
	            USART2_commandlong_old = USART2_commandlong;
	            Delay_Ms(50);
	        }
	    }

	    if(showTimeCnt > 0){
            showTimeCnt --;
            if(showTimeCnt == 0){
                OLED_Clear();
                sprintf(contStr, "   Code:");

                OLED_ShowString(0, 0, titleStr);
                OLED_ShowString(0, 3, title2Str);
                OLED_ShowString(0, 6, contStr);
            }
            Delay_Ms(1);
        }



	}
}


/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {


      USART2_RecieveBuff[USART2_commandlong++] = USART_ReceiveData(USART2);

//    if(RxCnt1 == TxSize2)
//    {
//      USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
//            Rxfinish1=1;
//    }
  }

}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    RxBuffer2[RxCnt2++] = USART_ReceiveData(USART3);

    if(RxCnt2 == TxSize1)
    {
      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
            Rxfinish2=1;
    }
  }
}

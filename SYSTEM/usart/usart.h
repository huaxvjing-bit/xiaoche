#ifndef __USART_H
#define __USART_H

#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  200  	// 最大接收字节数
#define EN_USART1_RX   1		// 使能串口1接收

extern u8  USART_RX_BUF[USART_REC_LEN]; // 接收缓冲
extern u16 USART_RX_STA;         		// 接收状态标记	
extern u8  count;
extern volatile u8 Remote_Cmd_Received;

void uart_init(u32 bound);

#endif


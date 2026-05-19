#include "bsp_uart.h"



void usart0_debug_init(uint32_t bound)// pcb板tx，rx注意，可能有问题
{
    debug_init(); 
}
void usart0_debug_sendbuf(const uint8_t *pData, uint16_t Size)
{
    uart_write_buffer(UART_0,pData,Size);
}


uint8 data_to_send[50];	//发送数据缓存


void Send_User8bite(uint8 channel,uint8 data)
{
        
	uint8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=channel;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=data;
	
        data_to_send[3]=_cnt-4;
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
        usart0_debug_sendbuf(data_to_send,_cnt);
}
void Send_User16bite_int(uint8 channel,int16 data)
{
        uint8_t *byte_ptr = (uint8_t*)&data;
	uint8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=channel;
	data_to_send[_cnt++]=0x00;
        data_to_send[_cnt++]=byte_ptr[1];
        data_to_send[_cnt++]=byte_ptr[0];
	
        data_to_send[3]=_cnt-4;
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
        usart0_debug_sendbuf(data_to_send,_cnt);
}
void Send_User32bite_float(uint8 channel,float data)
{
        uint8_t *byte_ptr = (uint8_t*)&data;
	uint8 _cnt=0;
	
        data_to_send[_cnt++]=0xAA;
        data_to_send[_cnt++]=0xAA;
        data_to_send[_cnt++]=channel;
        data_to_send[_cnt++]=0x00;
        data_to_send[_cnt++]=byte_ptr[3];
        data_to_send[_cnt++]=byte_ptr[2];
        data_to_send[_cnt++]=byte_ptr[1];
        data_to_send[_cnt++]=byte_ptr[0];
	
        data_to_send[3]=_cnt-4;
	uint8 sum = 0;
	for(uint8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
        usart0_debug_sendbuf(data_to_send,_cnt);
}





void usart1_wireless_init(uint32_t bound)// pcb板tx，rx注意，可能有问题
{
    uart_init(UART_1, bound, UART1_TX_P04_1, UART1_RX_P04_0,8,CY_SCB_UART_PARITY_NONE,CY_SCB_UART_STOP_BITS_1);
    uart_rx_interrupt(UART_1, 1); 
    
}
void usart1_wireless_sendbuf(const uint8_t *pData, uint16_t Size)
{
    uart_write_buffer(UART_1,pData,Size);
}


void usart2_exti_init(uint32_t bound)
{
  
    uart_init(UART_2, bound, UART2_TX_P10_1, UART2_RX_P10_0,8,CY_SCB_UART_PARITY_NONE,CY_SCB_UART_STOP_BITS_1);
//    uart_rx_interrupt(UART_2, 1);                                           // 开启 UART_INDEX 的接收中断
    
    
}
void usart2_exti_sendbuf(const uint8_t *pData, uint16_t Size)
{
    uart_write_buffer(UART_2,pData,Size);
}


void usart4_rs485_init(uint32_t bound)
{
    gpio_init(P17_1, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 485控制引脚，高电平发射，低电平接收
    uart_init(UART_4, bound, UART4_TX_P14_1, UART4_RX_P14_0,8,CY_SCB_UART_PARITY_NONE,CY_SCB_UART_STOP_BITS_1);
    uart_rx_interrupt(UART_4, 1); 
}

void usart4_rs485_sendbuf(const uint8_t *pData, uint16_t Size)
{
    gpio_set_level(P17_4,1);
    uart_write_buffer(UART_4,pData,Size);
}

// 调试串口0
SerialInterface_t UserDev_usart0 = {
	.init = usart0_debug_init,
	.transmit = usart0_debug_sendbuf,
};

// 无线串口1
SerialInterface_t UserDev_usart1 = {
	.init = usart1_wireless_init,
	.transmit = usart1_wireless_sendbuf,
};

// 外部串口2
SerialInterface_t UserDev_usart2 = {
	.init = usart2_exti_init,
	.transmit = usart2_exti_sendbuf,
};

// rs485串口4
SerialInterface_t UserDev_usart4 = {
	.init = usart4_rs485_init,
	.transmit = usart4_rs485_sendbuf,
};
DebugInterface_t UserDev_Debug_ANOTC = {
        .transmit_32biteFloat= Send_User32bite_float,
        .transmit_16biteInit = Send_User16bite_int,
        .transmit_8biteUint = Send_User8bite,
};

  
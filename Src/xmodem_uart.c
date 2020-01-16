#include "xmodem_uart.h"
#include "quad_spi.h"

#define SOH     0x01
#define STX     0x02
#define EOT     0x04
#define ACK     0x06
#define NAK     0x15
#define CAN     0x18
#define MAXRETRANS  10

#define tx_usart(c)     do{while(!(UART7->ISR&(1<<USART_ISR_TXE_Pos)));UART7->TDR=c;}while(0)
#define tx_wait()   do while(!(UART7->ISR&(1<<USART_ISR_TC_Pos))); while(0)
#define flushinput()    do ; while(rx_usart(1000) >= 0)


uint8_t xbuff[3+1024+2+1];              //3 head chars + 1024 for XModem 1k + 2 crc + nul
uint8_t trychar='C',packetno=1,retry,retrans=MAXRETRANS,*p;
uint16_t bufsz;
uint32_t len=0;
int16_t data_rx;
uint32_t current_byte = 0;

__IO uint16_t timer1 = 0;
__IO uint16_t timer_UART_transmit = 0;
__IO uint8_t flag_UART_receive = 0;

extern __IO uint8_t flag_end_of_transmit;

void uart_xmodem_receive() {
	//if(!timer_UART_transmit) 
  {
    tx_usart(trychar);
    timer_UART_transmit = 1000;
  }
  //if(flag_UART_receive) 
  {
    flag_UART_receive = 0;
    if((data_rx = rx_usart(1000)) >= 0)
    {
      switch(data_rx)
      {
        case SOH:
            bufsz=128;
            start_recv();
            break;
        case STX:
            bufsz=1024;
            start_recv();
            break;
        case EOT:
            flushinput();
            tx_usart(ACK);
            packetno=1;
            len = 0;
            current_byte = 0;
            //redrawing image
            flag_end_of_transmit = 1;
            break;
            //normal end
        case CAN:
            if((data_rx=rx_usart(1000))==CAN)
            {
              flushinput();
              tx_usart(ACK);
              packetno=1;
              len = 0;
              current_byte = 0;
              //canceled by remote
            }
            break;
        default:
            break;
      }
    }
  }
}

void start_recv() {
    //trychar=0;
    p=xbuff;
    *p++=data_rx;
    for(uint16_t i = 0; i < (3 + bufsz + 1); i++)
    {
      if((data_rx = rx_usart(1000)) < 0) {
        flushinput();
        tx_usart(NAK);
        break;
      }
      *p++ = data_rx;
    }
    if(xbuff[1] == (uint8_t)(~xbuff[2]) && (xbuff[1]==packetno || xbuff[1]==(uint8_t)(packetno-1)) && check_crc(&xbuff[3],bufsz))
    {
      if(xbuff[1] == packetno)
      {
        uint32_t count = (800 * 480 * 3) - len;
        if(count > bufsz) 
          count = bufsz;
        if(count > 0)
          {
          for(uint32_t i = 0; i < count; i += 4)
          {
            uint32_t data=((uint32_t)xbuff[3+i+3]<<24)|((uint32_t)xbuff[3+i+2]<<16)|((uint32_t)xbuff[3+i+1]<<8)|((uint32_t)xbuff[3+i+0]<<0);
            //uint32_t index_data = 0;
            //if(packetno < 3)
            //index_data = i + (1024 * (packetno-1));

            //write in SDRAM
            *(__IO uint32_t*) (SDRAM_BANK_ADDR + current_byte) = (uint32_t)(data);

            current_byte += 4;
          }
            len+=count;
          }
        packetno++;
        retrans = MAXRETRANS + 1;
      }
      if(!--retrans)                    //too many retry error
      {
        flushinput();
        tx_usart(CAN);tx_usart(CAN);tx_usart(CAN);
        tx_usart('E');tx_usart('R');tx_usart('R');
      }
      tx_usart(ACK);
      return;
    }
    
//reject:
     flushinput();
     tx_usart(NAK);
}


static uint8_t check_crc(const uint8_t *buf,uint16_t size) 
{
    uint8_t c,i;
    uint16_t j,crc=0x0000;
    for(j = 0; j < size; j++)
    {
      c=*buf++;
      crc=crc^(c<<8);
      for(i=0;i<8;i++)
        if(crc&0x8000) crc=(crc<<1)^0x1021;
        else crc<<=1;
    }
    crc^=(*buf++)<<8;
    crc^=*buf++;
    return !crc;
}

static int16_t rx_usart(uint16_t ms)
{
    timer1 = ms;
    while(timer1)
      if(UART7->ISR & (1<<USART_ISR_RXNE_Pos))
        return UART7->RDR & 0xFF;
      else ;
    return -1;
}

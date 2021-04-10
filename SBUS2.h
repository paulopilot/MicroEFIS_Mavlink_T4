#ifndef SBUS2_H__
#define SBUS2_H__

#include <DMAChannel.h>

#define UART_RXBUFSIZE  30
#define SBUS_BAUD       100000

DMAChannel dmaRX(false);

static volatile uint8_t   rxbuf[UART_RXBUFSIZE];
    static volatile uint8_t   sbusData[UART_RXBUFSIZE];
    static volatile uint8_t   *dest;
    static volatile bool      telemetry_ready;
    static volatile bool      sbus_ready;
    static volatile bool      frame_ready;
    static volatile uint8_t   FER_buf[100];
    static volatile uint8_t   FER_count;

class SBUS2{
  public:
    SBUS2(HardwareSerial& bus);
    void begin();
    ~SBUS2();
  private:
    HardwareSerial* _bus;
    
    void dmaRXinit();
    static void dmaRXisr(void);
    static volatile uint8_t   rxbuf[UART_RXBUFSIZE];
    static volatile uint8_t   sbusData[UART_RXBUFSIZE];
    static volatile uint8_t   *dest;
    static volatile bool      telemetry_ready;
    static volatile bool      sbus_ready;
    static volatile bool      frame_ready;
    static volatile uint8_t   FER_buf[100];
    static volatile uint8_t   FER_count;    
};


/* SBUS object, input the serial bus */
SBUS2::SBUS2(HardwareSerial& bus)
{
  _bus = &bus;
  telemetry_ready = false;
  sbus_ready = false;
  frame_ready = false;
  FER_count = 0;
}

/* handle interrupt DMA serial communication */
static void SBUS2::dmaRXisr(void)
{
  uint32_t daddr;  

  dmaRX.clearInterrupt();  
  daddr = (uint32_t)(dmaRX.TCD->DADDR);
  
  if (daddr < (uint32_t)rxbuf + sizeof(rxbuf) / 2)
  {  
    dest = &rxbuf[UART_RXBUFSIZE / 2]; 
  }else{ 
    dest = rxbuf;
  }
  debug.println("ISR SBUS");
}

/* initialize DMA serial communication */
void SBUS2::dmaRXinit()
{
  dmaRX.begin(true); // Allocate the DMA channel
  dmaRX.source((uint8_t &) LPUART4_DATA); // SERIAL2 = LPUART4 ! ! !
  dmaRX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART4_RX);
  //dmaRX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART4_TX);
  dmaRX.destinationBuffer(rxbuf, sizeof(rxbuf));
  dmaRX.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
  dmaRX.enable();
  dmaRX.attachInterrupt(dmaRXisr);
}

/* starts the serial communication */
void SBUS2::begin()
{
#if defined(__IMXRT1062__)
  _bus->begin(SBUS_BAUD, SERIAL_8E2);
  dmaRXinit();
#endif
}
#endif

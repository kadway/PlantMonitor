//--------------------------------------------------------------
// File     : stm32_ub_uart.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_UART_H
#define __STM32F4_UB_UART_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "stm32f4xx.h"



//--------------------------------------------------------------
// List all UARTs
// (no duplicate numbers and starting from 0)
//--------------------------------------------------------------

typedef enum
{

  COM2 = 0,  // COM2 (TX=PA2, RX=PA3)
  COM3 = 1   // COM3 (TX=PD8, RX=PD9)
 //To Fix: UART 1 was not printing anything out
 //COM1 = 2  // COM1 (TX=PA9, RX=PA10)
}UART_NAME_t;

#define  UART_NUM   1 // Number of used UART from UART_NAME_t


//--------------------------------------------------------------
// Send end identifier
//--------------------------------------------------------------
typedef enum {
  NONE = 0,  // No Identifier
  LFCR,      // LineFeed + CarriageReturn (0x0A,0x0D)
  CRLF,      // CarriageReturn + LineFeed (0x0D,0x0A)
  LF,        // only LineFeed (0x0A)
  CR         // only CarriageReturn (0x0D)
}UART_LASTBYTE_t;


//--------------------------------------------------------------
// Receive status
//--------------------------------------------------------------
typedef enum {
  RX_EMPTY = 0,  // nothing received
  RX_READY,      // there is something in receive buffer
  RX_FULL        // receive buffer is full
}UART_RXSTATUS_t;


//--------------------------------------------------------------
// Structure of UART-Pins
//--------------------------------------------------------------
typedef struct {
  GPIO_TypeDef* PORT;     // Port
  const uint16_t PIN;     // Pin
  const uint32_t CLK;     // Clock
  const uint8_t SOURCE;   // Source
}UART_PIN_t;

//--------------------------------------------------------------
// Struktur eines UARTs
//--------------------------------------------------------------
typedef struct {
  UART_NAME_t UART_NAME;    // Name
  const uint32_t CLK;       // Clock
  const uint8_t AF;         // AF
  USART_TypeDef* UART;      // UART
  const uint32_t BAUD;      // Baudrate
  const uint8_t INT;        // Interrupt
  UART_PIN_t TX;            // TX-Pin
  UART_PIN_t RX;            // RX-Pin
}UART_t;


//--------------------------------------------------------------
// Defines for receiving
//--------------------------------------------------------------
#define  RX_BUF_SIZE   50    // Size of RX-Buffer in Bytes
#define  RX_FIRST_CHR  0x20  // erstes erlaubte Zeichen (Ascii-Wert)
#define  RX_LAST_CHR   0x7E  // letztes erlaubt Zeichen (Ascii-Wert)
#define  RX_END_CHR    0x0D  // Endekennung (Ascii-Wert)


//--------------------------------------------------------------
// Struktur fuer UART_RX
//--------------------------------------------------------------
typedef struct {
  char rx_buffer[RX_BUF_SIZE]; // RX-Puffer (Ringpuffer)
  uint16_t wr_ptr;             // Schreib Pointer
  uint16_t rd_ptr;             // Lese Pointer
  UART_RXSTATUS_t status;      // RX-Status
}UART_RX_t;
UART_RX_t UART_RX[UART_NUM];


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------

void UB_Uart_Init(void);
void UB_Uart_SendByte(UART_NAME_t uart, uint16_t wert);
void UB_Uart_SendString(UART_NAME_t uart, char *ptr, UART_LASTBYTE_t end_cmd);
UART_RXSTATUS_t UB_Uart_ReceiveString(UART_NAME_t uart, char *ptr);
void UB_Uart_SendArray(UART_NAME_t uart, uint8_t *data, uint16_t cnt);
uint32_t UB_Uart_ReceiveArray(UART_NAME_t uart, uint8_t *data);


//--------------------------------------------------------------
#endif // __STM32F4_UB_UART_H

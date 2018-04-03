/****************************************************************************************************/
/**
\file       linif.c
\brief      Mcu Abstraction level - LIN interface
\author     Juan Pablo Garcia
\version    1.0
\project    LIN
\date       22/March/2018
*/
/****************************************************************************************************/

/********************************************************************************
 *                               Include Files
 ********************************************************************************/
#include "linif.h"
#include "board.h"
#include "uart.h"

 /*******************************************************************************
 *                               Macro Definitions
 ********************************************************************************/

 /*******************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/
LinStateType LinState;
uint8_t PidCommand;
uint16_t Baudrate;
Uart *pUart = LIN_BASE_UART;
/********************************************************************************
*                               Static Function Declarations
********************************************************************************/
void Lin_Isr(void);

/********************************************************************************
 *                               Global and Static Function Definitions
 ********************************************************************************/

 /*****************************************************************************
 *
 * Function:        Lin_Init
 *
 * Description:     - This function will configure the lower layer UART driver
 *                  - Baudrate will as per LinBaudrate parameter
 *                  - Interrupts shall be configured for each data byte to be transmitted or received
 *                  - This function shall provide to the lower layer a function callback (Lin_Isr) to be 
 *                    invoked at any of the RX or TX UART interrupts
 *
 * Caveats:
 *   None
 *
 *****************************************************************************/
void Lin_Init ( const LinConfigType* Config)
{
    LinState = IDLE;
    //To complete
    //Baudrate= LinBaudrate;
    const Pin pPins[] = LIN_UART_PINS;
    PIO_Configure(pPins, PIO_LISTSIZE(pPins));
    PMC_EnablePeripheral(LIN_BASE_ID);
    //Config_Uart(LinBaudrate);
    UART_Configure(pUart, UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO | UART_MR_BRSRCCK_PERIPH_CLK,
                   Baudrate, BOARD_MCK, Lin_Isr);
    /* Clear pending IRQs and Set priority of IRQs */
  
    NVIC_ClearPendingIRQ(LIN_BASE_IRQ);
  	NVIC_SetPriority(LIN_BASE_IRQ, 1);    
    ///Done in uart.c UART_Configure()//pUart->UART_CR = UART_CR_RXEN ;//
    UART_EnableIt(pUart, UART_IER_TXRDY);
	  /* Enable interrupt  */
  	NVIC_EnableIRQ(LIN_BASE_IRQ);
}

 /*****************************************************************************
 *
 * Function:        Lin_SendFrame
 *
 * Description:     - This function will send a predefined header as per the LIN protocol with the rate define 
 *                    in the Lin_Init function.
 *                  - This function shall be asynchronous, i.e. it will trigger the "send command" and will 
 *                    continue its operation without waiting for the header to be completely sent over the bus.
 *
 * Caveats:
 *   None
 *
 *****************************************************************************/
Std_ReturnType Lin_SendFrame (uint8_t LinPid, LinPduType* PduInfoPtr )
{           
	if(LinState == IDLE )//&& LinPid != NO_CMD)
	{
      
    	PidCommand = LinPid;
    	LinState = SEND_BREAK;
      /*Hard call to ISR handler to process first part of LIN header, otherwise it cannot be *
      * activated on the first instance*/
      Lin_Isr();
	}
}

void Lin_Isr(void)
{
   UART_DisableIt(pUart, UART_IDR_TXEMPTY);
   UART_DisableIt(pUart, UART_IDR_TXRDY);
   UART_SetTransmitterEnabled(pUart, 0);

  switch(LinState)
	{
		case SEND_BREAK:
      /*Sending Break */
      /*Configre new baudrate to make a larger stop in order to accomplish the Break time*/
      UART_UpdateBaudRate(pUart, (BOARD_MCK / (Baudrate*9/14)) / 16);
			UART_PutCharIT(pUart, BREAK_CMD);
			LinState = SEND_SYNC;
		break;

		case SEND_SYNC:
      /*Sending Sync*/
      UART_UpdateBaudRate(pUart, (BOARD_MCK / (Baudrate)) / 16);
    	UART_PutCharIT(pUart, SYNC_CMD);
			LinState = SEND_PID;
		break;

		case SEND_PID:
      /*Sending Pid*/
			UART_PutCharIT(pUart, PidCommand);
			LinState = IDLE;
		break;
  
		case SEND_RESPONSE:
		break;
		
		default:
			LinState = IDLE;
		break;
  }
}
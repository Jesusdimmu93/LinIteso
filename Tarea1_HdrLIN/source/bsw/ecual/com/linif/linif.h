/****************************************************************************************************/
/**
\file       linif.h
\brief      Mcu Abstraction level - LIN interface
\author     Juan Pablo Garcia
\version    1.0
\project    LIN
\date       22/March/2018
*/
/****************************************************************************************************/
#ifndef _LINIF_TYPES_
#define _LINIF_TYPES_
/********************************************************************************
 *                               Include Files
 ********************************************************************************/
#include "linif_Types.h"
#include "linif_cfg.h"

/********************************************************************************
*                               Macro Definitions
********************************************************************************/

#define LIN_BASE_UART       	UART2
#define LIN_BASE_ID         	ID_UART2
#define LIN_BASE_IRQ        	UART2_IRQn   //UART2_Handler
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define LIN_UART_PINS        	{PINS_UART2}

#define BREAK_CMD  0x00
#define SYNC_CMD   0x55

/********************************************************************************
*                               Type Definitions
********************************************************************************/

/********************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/

/********************************************************************************
*                               Function Declarations
********************************************************************************/

extern void Lin_Init ( const LinConfigType* Config);
extern Std_ReturnType Lin_SendFrame ( uint8_t Channel, LinPduType* PduInfoPtr );
extern Std_ReturnType Lin_CRC_Calculation (LinPduType* LinCRC);
extern Std_ReturnType Lin_GetSlaveResponse ( uint8_t Channel, uint8_t** LinSduPtr );
void LIN_UpdateBaudRate(uint8_t Channel,uint32_t baudrate);
void LIN_SetTransmitterEnabled(uint8_t Channel, uint8_t enabled);
void LIN_PutChar( uint8_t Channel, uint8_t c);
void LIN_EnableIt(uint8_t Channel, uint32_t mode);
#endif
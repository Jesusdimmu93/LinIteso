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

/********************************************************************************
*                               Type Definitions
********************************************************************************/
typedef struct 
{
    Uart *uart;
    uint8_t ID;
    uint8_t IRQn;
    const Pin ptrPins[1];
}UartChannelsType;

typedef struct 
{
    uint16_t BaudRate;
    LinStateType LinState;
}LinCtlType;

 /*******************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/
LinStateType LinState;

UartChannelsType UartChannel[3] = 
{
    {UART0, ID_UART0, UART0_IRQn, PINS_UART0},
    {UART2, ID_UART2, UART2_IRQn, PINS_UART2},
    {UART4, ID_UART4, UART4_IRQn, PINS_UART4},
};

LinCtlType LinCtlChnl[3];
LinPduType LinPduFrame;
uint8_t LinTXCounter;
/********************************************************************************
*                               Static Function Declarations
********************************************************************************/
void Lin_Isr(uint8_t Channel);

/********************************************************************************
*                               Global and Static Function Definitions
********************************************************************************/

/*****************************************************************************
*
* Function:        Lin_Init
*
* Description:     This function shall initialize the LIN module as well as the LIN channels. 
*                  Note that different sets of configuration may be provided.
*                  Initialization shall be according to the configuration set pointed by the 
*                  parameter Config.
*
* Caveats:
*   None
*
*****************************************************************************/
void Lin_Init ( const LinConfigType* Config)
{
    uint8_t Chnl_Idx;
    uint8_t Chnl_ID;
    if(Config != NULL)
    {
        for(Chnl_Idx = 0;Chnl_Idx < Config->LinNumberOfChannels; Chnl_Idx++)
        {
            Chnl_ID = Config->LinChannel[Chnl_Idx].LinChannelId;
            LinState = IDLE;
            LinCtlChnl[Chnl_Idx].LinState = IDLE;
            LinCtlChnl[Chnl_Idx].BaudRate = Config->LinChannel[Chnl_ID].LinChannelBaudrate;
            PIO_Configure(UartChannel[Chnl_ID].ptrPins, PIO_LISTSIZE(UartChannel[Chnl_ID].ptrPins));
            PMC_EnablePeripheral(UartChannel[Chnl_ID].ID);
            UART_Configure(UartChannel[Chnl_ID].uart, UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO | UART_MR_BRSRCCK_PERIPH_CLK,
                           LinCtlChnl[Chnl_ID].BaudRate, BOARD_MCK, Lin_Isr);
            /* Clear pending IRQs and Set priority of IRQs */
            NVIC_ClearPendingIRQ(UartChannel[Chnl_ID].IRQn);
            NVIC_SetPriority(UartChannel[Chnl_ID].IRQn, 1);    
            /* Enable interrupt  */
            NVIC_EnableIRQ(UartChannel[Chnl_ID].IRQn);
        }
    }
}

/*****************************************************************************
*
* Function:        Lin_SendFrame
*
* Description:     The function Lin_SendFrame shall send the header part (Break Field, 
*                  Synch Byte Field and PID Field) and, depending on the direction of 
*                  the frame response, a complete LIN response part of a LIN frames on 
*                  the addressed LIN channel.
*                  In case of receiving data the LIN Interface has to wait for the 
*                  corresponding response part of the LIN frame by polling with the 
*                  function Lin_GetSlaveResponse() after using the function Lin_SendFrame().
*                  This function shall calculate the LIN Protected ID accordingly.
* Caveats:
*   None
*
*****************************************************************************/
Std_ReturnType Lin_SendFrame ( uint8_t Channel, LinPduType* PduInfoPtr)
{
    Std_ReturnType RtnVal = E_NOT_OK;
    if(LinCtlChnl[Channel].LinState == IDLE )
    {
        //PidCommand = LinPid;
        LinCtlChnl[Channel].LinState = SEND_BREAK;
        /*Hard call to ISR handler to process first part of LIN header, otherwise it cannot be *
        activated on the first instance*/
        Lin_Isr(Channel);
        RtnVal = E_OK;
    }
    return RtnVal;
}

/*****************************************************************************
*
* Function:        Lin_SendFrame
*
* Description:     This function shall return the current reception status of the 
*                  LIN driver. If a SDU has been successfully received, the 
*                  LinGetSlaveResponse shall store the SDU in a buffer referenced by 
*                  LinSduPtr. The buffer will only be valid and shall be read until 
*                  the next Lin_SendFrame function call.
*
* Caveats:
*   None
*
*****************************************************************************/
Std_ReturnType Lin_GetSlaveResponse ( uint8_t Channel, uint8_t** LinSduPtr )
{

}

Std_ReturnType Lin_CRC_Calculation (LinPduType* LinCRC)
{

}

/*En un punto se tendra que cambiar el argumento a tipo Uart * y buscar su canal virtual*/
void Lin_Isr(uint8_t Channel)
{
    //UART_DisableIt(pUart, UART_IDR_TXEMPTY);
    //UART_DisableIt(pUart, UART_IDR_TXRDY);
    //UART_SetTransmitterEnabled(pUart, 0);  /*Disable UART*/

    switch(LinCtlChnl[Channel].LinState)
    {
        case SEND_BREAK:
            /*Sending Break */
            /*Configre new baudrate to make a larger stop in order to accomplish the Break time*/
            LIN_UpdateBaudRate(Channel, ((uint32_t)LinCtlChnl[Channel].BaudRate*5)/8);
            LIN_SetTransmitterEnabled(Channel,(uint8_t) 1); /*Enable UART*/
            LIN_PutChar(Channel, BREAK_CMD);
            LinState = SEND_SYNC;
            LIN_EnableIt(Channel, UART_IER_TXEMPTY);
        break;

        case SEND_SYNC:
              /*Update Baud rate*/
              LIN_UpdateBaudRate(Channel, (uint32_t)LinCtlChnl[Channel].BaudRate);
              /*Sending Sync*/
              LIN_SetTransmitterEnabled(Channel,(uint8_t) 1); /*Enable UART*/
              LIN_PutChar(Channel, SYNC_CMD);
              LinState = SEND_PID;
              LIN_EnableIt(Channel, UART_IER_TXEMPTY);
        break;
    
        case SEND_PID:
            /*Sending Pid*/
            LIN_SetTransmitterEnabled(Channel,(uint8_t) 1); /*Enable UART*/
            LIN_PutChar(Channel,(uint8_t) LinPduFrame.Pid );
            if(LinPduFrame.Drc==LIN_MASTER_RESPONSE)
            {
                LinState = IDLE;
            }
            else if(LinPduFrame.Drc==LIN_SLAVE_RESPONSE)
            {
                LinState = SEND_RESPONSE;
                LinTXCounter=LinPduFrame.Dl;
            }
            LIN_EnableIt(Channel, UART_IER_TXEMPTY);
        break;

        case SEND_RESPONSE:
            LIN_SetTransmitterEnabled(Channel,(uint8_t) 1); /*Enable UART*/
            if(LinTXCounter==0)
            {
                LinState = SEND_CHKSUM;
            }
            else
            {
                LIN_PutChar(Channel,(uint8_t) LinPduFrame.SduPtr[LinPduFrame.Dl-LinTXCounter] );
                LinTXCounter--;
                /*Nothing Yet**/
            }
            LIN_EnableIt(Channel, UART_IER_TXEMPTY);
        break;

        case SEND_CHKSUM:
            LIN_UpdateBaudRate(Channel, (uint32_t)LinCtlChnl[Channel].BaudRate);
            LIN_SetTransmitterEnabled(Channel,(uint8_t) 1); /*Enable UART*/
            LIN_PutChar(Channel, SYNC_CMD);
            LinState = IDLE;
            LIN_EnableIt(Channel, UART_IER_TXEMPTY);
        break;

        default:
            LinState = IDLE;
        break;
    }
}

void LIN_UpdateBaudRate(uint8_t Channel,uint32_t baudrate)
{
    UartChannel[Channel].uart->UART_BRGR = (BOARD_MCK / baudrate) / 16;
}

void LIN_SetTransmitterEnabled(uint8_t Channel, uint8_t enabled)
{
	if (enabled) {
		UartChannel[Channel].uart->UART_CR = UART_CR_TXEN;
	} else {
		UartChannel[Channel].uart->UART_CR = UART_CR_TXDIS;
	}
}

void LIN_PutChar( uint8_t Channel, uint8_t c)
{
	/* Wait for the transmitter to be ready*/
	//while (!UART_IsRxReady(uart) && !UART_IsTxSent(uart));

	/* Send character*/
	UartChannel[Channel].uart->UART_THR = c;

	/* Wait for the transfer to complete*/
	//while (!UART_IsTxSent(uart));
}

void LIN_EnableIt(uint8_t Channel, uint32_t mode)
{
	UartChannel[Channel].uart->UART_IER = mode;
}
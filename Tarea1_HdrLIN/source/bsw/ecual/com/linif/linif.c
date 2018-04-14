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
    LinPduType LinPduFrame;
    uint8_t CheckSum;
    uint8_t LinTxCnt;
}LinCtlType;

 /*******************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/

UartChannelsType UartChannel[3] = 
{
    {UART0, ID_UART0, UART0_IRQn, PINS_UART0},
    {UART2, ID_UART2, UART2_IRQn, PINS_UART2},
    {UART4, ID_UART4, UART4_IRQn, PINS_UART4},
};

LinCtlType LinCtlChnl[3];

/********************************************************************************
*                               Static Function Declarations
********************************************************************************/
void Lin_Isr(uint8_t Channel);
void Lin_CheckSum (uint8_t Channel);

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
            LinCtlChnl[Chnl_ID].LinState = IDLE;
            LinCtlChnl[Chnl_ID].BaudRate = Config->LinChannel[Chnl_Idx].LinChannelBaudrate;
            LinCtlChnl[Chnl_ID].CheckSum = 0xFF;
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
        LinCtlChnl[Channel].LinPduFrame = *PduInfoPtr;
        if(LinCtlChnl[Channel].LinPduFrame.Drc == LIN_MASTER_RESPONSE)
        {
            Lin_CheckSum(Channel);
        }
        
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
* Function:        Lin_GetSlaveResponse
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
  uint8_t Crc_Byte;
  UART_ReceiveBuffer(UartChannel[Channel].uart, *LinSduPtr, LinCtlChnl[Channel].LinPduFrame.Dl);
  /*Read/Send CRC*/
  Crc_Byte = UART_GetChar(UartChannel[Channel].uart);
  LinCtlChnl[Channel].LinState = IDLE;  
}

void Lin_CheckSum (uint8_t Channel)
{
    uint8_t Data_Idx;
    uint16_t ChckSm;

    if(LinCtlChnl[Channel].LinPduFrame.Cs == LIN_ENHANCED_CS)
    {
        ChckSm = (uint16_t)LinCtlChnl[Channel].LinPduFrame.Pid;
    }
    else
    {
        ChckSm = 0u;
    }

    for(Data_Idx = 0; Data_Idx < LinCtlChnl[Channel].LinPduFrame.Dl; Data_Idx++)
    {
        ChckSm +=LinCtlChnl[Channel].LinPduFrame.SduPtr[Data_Idx];
        if(ChckSm & 0x0100)
        {
            ChckSm &= 0x00FF;
            ChckSm++;
        }
    }

     LinCtlChnl[Channel].CheckSum =~((uint8_t)ChckSm);
}

/*En un punto se tendra que cambiar el argumento a tipo Uart * y buscar su canal virtual*/
void Lin_Isr(uint8_t Channel)
{
    UART_DisableIt(UartChannel[Channel].uart, UART_IDR_TXEMPTY);
    //UART_DisableIt(UartChannel[Channel].uart, UART_IDR_TXRDY);
    UART_SetTransmitterEnabled(UartChannel[Channel].uart, 0);  /*Disable UART*/

    switch(LinCtlChnl[Channel].LinState)
    {
        case SEND_BREAK:
            /*Sending Break */
            /*Configre new baudrate to make a larger stop in order to accomplish the Break time*/
            UART_UpdateBaudRate(UartChannel[Channel].uart, ((uint32_t)LinCtlChnl[Channel].BaudRate*5)/8);
            UART_SetTransmitterEnabled(UartChannel[Channel].uart,(uint8_t) 1); /*Enable UART*/
            UART_PutCharAsync(UartChannel[Channel].uart, BREAK_CMD);
            LinCtlChnl[Channel].LinState = SEND_SYNC;
            UART_EnableIt(UartChannel[Channel].uart, UART_IER_TXEMPTY);
        break;

        case SEND_SYNC:
              /*Update Baud rate*/
              UART_UpdateBaudRate(UartChannel[Channel].uart, (uint32_t)LinCtlChnl[Channel].BaudRate);
              /*Sending Sync*/
              UART_SetTransmitterEnabled(UartChannel[Channel].uart,(uint8_t) 1); /*Enable UART*/
              UART_PutCharAsync(UartChannel[Channel].uart, SYNC_CMD);
              LinCtlChnl[Channel].LinState = SEND_PID;
              UART_EnableIt(UartChannel[Channel].uart, UART_IER_TXEMPTY);
        break;
    
        case SEND_PID:
            /*Sending Pid*/
            UART_SetTransmitterEnabled(UartChannel[Channel].uart,(uint8_t) 1); /*Enable UART*/
            UART_PutCharAsync(UartChannel[Channel].uart,(uint8_t) LinCtlChnl[Channel].LinPduFrame.Pid );
            if(LinCtlChnl[Channel].LinPduFrame.Drc==LIN_MASTER_RESPONSE)
            {
                LinCtlChnl[Channel].LinState = SEND_RESPONSE;
                LinCtlChnl[Channel].LinTxCnt = 0;//LinCtlChnl[Channel].LinPduFrame.Dl;
            }
            else if(LinCtlChnl[Channel].LinPduFrame.Drc==LIN_SLAVE_RESPONSE)
            {
                LinCtlChnl[Channel].LinState = READ_RESPONSE;
            }
            UART_EnableIt(UartChannel[Channel].uart, UART_IER_TXEMPTY);
        break;

        case SEND_RESPONSE:
            UART_SetTransmitterEnabled(UartChannel[Channel].uart,(uint8_t) 1); /*Enable UART*/

            UART_PutCharAsync(UartChannel[Channel].uart,(uint8_t) LinCtlChnl[Channel].LinPduFrame.SduPtr[LinCtlChnl[Channel].LinTxCnt]);
            LinCtlChnl[Channel].LinTxCnt++;
            if(LinCtlChnl[Channel].LinTxCnt >= LinCtlChnl[Channel].LinPduFrame.Dl)
            {
                LinCtlChnl[Channel].LinTxCnt = 0;
                LinCtlChnl[Channel].LinState = SEND_CHKSUM;
            }
            UART_EnableIt(UartChannel[Channel].uart, UART_IER_TXEMPTY);
        break;
        
        case READ_RESPONSE:
             //Response will be read synchronously
          break;

        case SEND_CHKSUM:
            //LIN_UpdateBaudRate(Channel, (uint32_t)LinCtlChnl[Channel].BaudRate);
            UART_SetTransmitterEnabled(UartChannel[Channel].uart,(uint8_t) 1); /*Enable UART*/
            /*include here real checkSum*/
            UART_PutCharAsync(UartChannel[Channel].uart, 0x12);
            LinCtlChnl[Channel].LinState = IDLE;
            UART_EnableIt(UartChannel[Channel].uart, UART_IER_TXEMPTY);
        break;

        default:
            LinCtlChnl[Channel].LinState = IDLE;
        break;
    }
}

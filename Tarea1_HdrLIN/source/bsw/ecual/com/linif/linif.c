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
    LinStateType LinState;
    LinPduType LinPduFrame;
    uint8_t Pid;
    uint8_t CheckSum;
    uint8_t LinTxCnt;
    uint8_t Buffer[8];
    uint8_t RxFinished;
    uint8_t RxRead;
}LinCtlType;

 /*******************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/

UartChannelsType UartChannel[MAX_NUM_UART_CHANNELS] = 
{
    {UART0, ID_UART0, UART0_IRQn, PINS_UART0},
    {UART1, ID_UART1, UART1_IRQn, PINS_UART1},  /*Not available*/
    {UART2, ID_UART2, UART2_IRQn, PINS_UART2},
    {UART3, ID_UART3, UART3_IRQn, PINS_UART3},  /*Not available*/
    {UART4, ID_UART4, UART4_IRQn, PINS_UART4},
};

uint8_t Phy_to_Logic_Chnl[MAX_NUM_UART_CHANNELS];

LinCtlType *LinCtlChnl;

const LinConfigType *LinCfgGlb;
/********************************************************************************
*                               Static Function Declarations
********************************************************************************/
void Lin_Isr(uint8_t Channel);
static void Lin_Parity (uint8_t Channel);
static uint8_t Lin_CheckSum (uint8_t Channel);

/********************************************************************************
*                               Global and Static Function Definitions
********************************************************************************/

static void Lin_Parity(uint8_t Channel)
{
    uint8_t Pid_temp,P1,P0;

    Pid_temp = LinCtlChnl[Channel].LinPduFrame.Pid;
    P0 = ((Pid_temp >>  ID0_MASK_POS) & 0x01);
    P0 ^= ((Pid_temp >> ID1_MASK_POS) & 0x01);
    P0 ^= ((Pid_temp >> ID2_MASK_POS) & 0x01);
    P0 ^= ((Pid_temp >> ID4_MASK_POS) & 0x01);

    P1 = ((Pid_temp >>  ID1_MASK_POS) & 0x01);
    P1 ^= ((Pid_temp >> ID3_MASK_POS) & 0x01);
    P1 ^= ((Pid_temp >> ID4_MASK_POS) & 0x01);
    P1 ^= ((Pid_temp >> ID5_MASK_POS) & 0x01);
    P1 = ~P1;

    Pid_temp &= D_MASK;
    Pid_temp |= (P0 << PO_MASK_POS) | (P1 << P1_MASK_POS);
    LinCtlChnl[Channel].Pid = Pid_temp;
}

static uint8_t Lin_CheckSum (uint8_t Channel)
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

     return ~((uint8_t)ChckSm);
}

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
    uint8_t PhyChnl_ID;
    Uart * myUart;
    if(Config != NULL)
    {
        LinCfgGlb = Config;
        /*malloc does not requires a casting for the return pointer*/
        LinCtlChnl = malloc(sizeof(LinCtlType) * LinCfgGlb->LinNumberOfChannels);

        for(Chnl_Idx = 0;Chnl_Idx < LinCfgGlb->LinNumberOfChannels; Chnl_Idx++)
        {
            PhyChnl_ID = LinCfgGlb->LinChannel[Chnl_Idx].LinChannelId;
            myUart = UartChannel[PhyChnl_ID].uart;

            Phy_to_Logic_Chnl[PhyChnl_ID] = Chnl_Idx;
            LinCtlChnl[Chnl_Idx].LinState = IDLE;
            LinCtlChnl[Chnl_Idx].CheckSum = 0xFF;
            LinCtlChnl[Chnl_Idx].RxFinished = TRUE;
            LinCtlChnl[Chnl_Idx].RxRead = TRUE;

            PIO_Configure(UartChannel[PhyChnl_ID].ptrPins, PIO_LISTSIZE(UartChannel[PhyChnl_ID].ptrPins));
            PMC_EnablePeripheral(UartChannel[PhyChnl_ID].ID);
            UART_Configure(myUart, UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO | UART_MR_BRSRCCK_PERIPH_CLK,
                           LinCfgGlb->LinChannel[Chnl_Idx].LinChannelBaudrate, BOARD_MCK, Lin_Isr);
            /* Clear pending IRQs and Set priority of IRQs */
            NVIC_ClearPendingIRQ(UartChannel[PhyChnl_ID].IRQn);
            NVIC_SetPriority(UartChannel[PhyChnl_ID].IRQn, 1);    
            /* Enable interrupt  */
            NVIC_EnableIRQ(UartChannel[PhyChnl_ID].IRQn);
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
    uint8_t LogicChannel;

    LogicChannel = Phy_to_Logic_Chnl[Channel];
    Std_ReturnType RtnVal = E_NOT_OK;
    if(LinCtlChnl[LogicChannel].LinState == IDLE && LinCtlChnl[LogicChannel].RxRead == TRUE)
    {
        //PidCommand = LinPid;
        LinCtlChnl[LogicChannel].LinPduFrame = *PduInfoPtr;
        Lin_Parity(LogicChannel);
        if(LinCtlChnl[LogicChannel].LinPduFrame.Drc == LIN_MASTER_RESPONSE)
        {
            LinCtlChnl[LogicChannel].CheckSum = Lin_CheckSum(LogicChannel);
        }
        else
        {
            LinCtlChnl[LogicChannel].RxRead = FALSE;
        }
            
        LinCtlChnl[LogicChannel].LinState = SEND_BREAK;
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
    Std_ReturnType retVal;
    uint8_t* tempPointer;
    if(LinCtlChnl[Channel].RxFinished == TRUE)
    {
        tempPointer = &LinCtlChnl[Channel].Buffer[0];
        LinSduPtr = &tempPointer;
        LinCtlChnl[Channel].RxRead = TRUE;
        retVal = E_OK;
    }
    else
    {
        retVal = E_NOT_OK;
    }

    return retVal;
  uint8_t Crc_Byte;

}

void Lin_Isr(uint8_t PhyChannel)
{
    uint8_t Channel;

    Channel = Phy_to_Logic_Chnl[PhyChannel];
    UART_DisableIt(UartChannel[PhyChannel].uart, UART_IDR_TXEMPTY);
    UART_DisableIt(UartChannel[PhyChannel].uart, UART_IDR_RXRDY);
    UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart, 0);  /*Disable UART*/

    switch(LinCtlChnl[Channel].LinState)
    {
        case SEND_BREAK:
            /*Sending Break */
            /*Configre new baudrate to make a larger stop in order to accomplish the Break time*/
            UART_UpdateBaudRate(UartChannel[PhyChannel].uart, ((uint32_t)LinCfgGlb->LinChannel[Channel].LinChannelBaudrate*5)/8);
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            UART_PutCharAsync(UartChannel[PhyChannel].uart, BREAK_CMD);
            LinCtlChnl[Channel].LinState = SEND_SYNC;
            UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_TXEMPTY);
        break;

        case SEND_SYNC:
              /*Update Baud rate*/
              UART_UpdateBaudRate(UartChannel[PhyChannel].uart, (uint32_t)LinCfgGlb->LinChannel[Channel].LinChannelBaudrate);
              /*Sending Sync*/
              UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
              UART_PutCharAsync(UartChannel[PhyChannel].uart, SYNC_CMD);
              LinCtlChnl[Channel].LinState = SEND_PID;
              UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_TXEMPTY);
        break;
    
        case SEND_PID:
            /*Sending Pid*/
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            UART_PutCharAsync(UartChannel[PhyChannel].uart,(uint8_t) LinCtlChnl[Channel].Pid );
            if(LinCtlChnl[Channel].LinPduFrame.Drc==LIN_MASTER_RESPONSE)
            {
                LinCtlChnl[Channel].LinState = SEND_RESPONSE;
                LinCtlChnl[Channel].LinTxCnt = 0;
                UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_TXEMPTY);
            }
            else if(LinCtlChnl[Channel].LinPduFrame.Drc==LIN_SLAVE_RESPONSE)
            {
                LinCtlChnl[Channel].LinState = READ_RESPONSE;
                LinCtlChnl[Channel].LinTxCnt = 0;
                LinCtlChnl[Channel].RxFinished = FALSE;
                
                UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_RXRDY);
            }
        break;

        case SEND_RESPONSE:
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            UART_PutCharAsync(UartChannel[PhyChannel].uart,(uint8_t) LinCtlChnl[Channel].LinPduFrame.SduPtr[LinCtlChnl[Channel].LinTxCnt]);
            LinCtlChnl[Channel].LinTxCnt++;
            if(LinCtlChnl[Channel].LinTxCnt >= LinCtlChnl[Channel].LinPduFrame.Dl)
            {
                LinCtlChnl[Channel].LinTxCnt = 0;
                LinCtlChnl[Channel].LinState = SEND_CHKSUM;
            }
            UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_TXEMPTY);
        break;
        
        case READ_RESPONSE:
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            LinCtlChnl[Channel].Buffer[LinCtlChnl[Channel].LinTxCnt] = UART_GetChar(UartChannel[PhyChannel].uart);
            LinCtlChnl[Channel].LinTxCnt++;
            if(LinCtlChnl[Channel].LinTxCnt >= LinCtlChnl[Channel].LinPduFrame.Dl)
            {
                LinCtlChnl[Channel].LinTxCnt = 0;
                LinCtlChnl[Channel].LinState = GET_CHKSUM;
                LinCtlChnl[Channel].RxFinished = TRUE;
            }
            UART_EnableIt(UartChannel[PhyChannel].uart, UART_IER_RXRDY);
          break;

        case GET_CHKSUM:
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            LinCtlChnl[Channel].CheckSum = UART_GetChar(UartChannel[PhyChannel].uart);
            if(LinCtlChnl[Channel].CheckSum == Lin_CheckSum(Channel))
            {
                /*Validate checksum*/
            }
            LinCtlChnl[Channel].LinState = IDLE;
          break;

        case SEND_CHKSUM:
            UART_SetTransmitterEnabled(UartChannel[PhyChannel].uart,(uint8_t) 1); /*Enable UART*/
            /*include here real checkSum*/
            UART_PutCharAsync(UartChannel[PhyChannel].uart, LinCtlChnl[Channel].CheckSum);
            LinCtlChnl[Channel].LinState = IDLE;
        break;

        default:
            LinCtlChnl[Channel].LinState = IDLE;
        break;
    }
}

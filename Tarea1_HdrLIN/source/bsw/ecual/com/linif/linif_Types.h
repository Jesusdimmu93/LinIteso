/****************************************************************************************************/
/**
\file       linif_cfg.h
\brief      Mcu Abstraction level - LIN interface types
\author     Juan Pablo Garcia
\version    1.0
\project    LIN
\date       22/March/2018
*/
/****************************************************************************************************/

/********************************************************************************
 *                               Include Files
 ********************************************************************************/
#include "Std_Types.h"
/********************************************************************************
*                               Macro Definitions
********************************************************************************/

/********************************************************************************
*                               Type Definitions
********************************************************************************/
typedef enum
{
    IDLE = 0,
    SEND_BREAK,
    SEND_SYNC,
    SEND_PID,
    SEND_RESPONSE,
    SEND_EOT
}LinStateType;

/*Specifies the Checksum model used in the LIN Frame*/
typedef enum
{
    LIN_ENHANCED_CS = 0,
    LIN_CLASSIC_CS
}LinFrameCsModelType;

/*Specifies whether the frame processor is required to transmit the response part of the LIN frame*/
typedef enum
{
    LIN_MASTER_RESPONSE = 0,
    LIN_SLAVE_RESPONSE
}LinFrameResponseType;

/*The LIN identifier (0..0x3F) along with its two parity bits*/
typedef uint8_t LinFramePidType;

/*Specifies the number of SDU of data bytes to copy*/
typedef uint8_t LinFrameDlType;

/*This type is used to provide PID, checksum model, data length and SDU pointer to the LIN driver*/
typedef struct 
{
    LinFramePidType        Pid;
    LinFrameCsModelType    Cs;
    LinFrameResponseType   Drc;
    LinFrameDlType         Dl;
    uint8_t*               SduPtr;
}LinPduType;


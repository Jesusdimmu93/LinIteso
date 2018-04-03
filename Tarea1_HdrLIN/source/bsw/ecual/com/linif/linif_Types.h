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
#include <stdio.h>
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

typedef struct
{
    uint8_t LinFramePid;
}LinFramePidType;

typedef enum
{
    LIN_ENHANCED_CS = 0,
    LIN_CLASSIC_CS
}LinFrameCsModelType;

typedef enum
{
    LIN_MASTER_RESPONSE = 0,
    LIN_SLAVE_RESPONSE
}LinFrameResponseType;

typedef enum
{
    LinFrameDl_1 = 1u,
    LinFrameDl_2,
    LinFrameDl_3,
    LinFrameDl_4,
    LinFrameDl_5,
    LinFrameDl_6,
    LinFrameDl_7,
    LinFrameDl_8
}LinFrameDlType;

typedef enum
{
    E_OK = 0u,
    E_NOK,
}Std_ReturnType;

  
  
typedef struct
{
    LinFramePidType       LinFramePidType_t;
    LinFrameCsModelType   LinFrameCsModelType_t;
    LinFrameResponseType  LinFrameResponseType;
    LinFrameDlType        LinFrameDlType;
    uint8_t*              LinFrameData;
}LinPduType;


typedef struct
{ 
    uint8_t               LinFrameData;
}LinConfigType;
    
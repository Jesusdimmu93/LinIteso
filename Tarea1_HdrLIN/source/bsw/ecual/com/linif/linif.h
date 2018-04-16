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
#ifndef LINIF_H
#define LINIF_H
/********************************************************************************
 *                               Include Files
 ********************************************************************************/
#include "linif_Types.h"
#include "linif_cfg.h"

/********************************************************************************
*                               Macro Definitions
********************************************************************************/

#define BREAK_CMD  0x00
#define SYNC_CMD   0x55

#define ID0_MASK_POS   0
#define ID1_MASK_POS   1
#define ID2_MASK_POS   2
#define ID3_MASK_POS   3
#define ID4_MASK_POS   4
#define ID5_MASK_POS   5
#define PO_MASK_POS    6
#define P1_MASK_POS    7
#define D_MASK         0x3F
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
extern Std_ReturnType Lin_GetSlaveResponse ( uint8_t Channel, uint8_t** LinSduPtr );
#endif
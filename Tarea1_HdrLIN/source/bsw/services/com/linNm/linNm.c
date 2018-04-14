/****************************************************************************************************/
/**
\file       linNm.c
\brief      Service level - LIN network management
\author     Juan Pablo Garcia
\version    1.0
\project    Lin Drv
*/
/****************************************************************************************************/

/********************************************************************************
*                               Include Files
********************************************************************************/
#include "linNm.h"
#include "linif.h"
/*******************************************************************************
*                               Macro Definitions
********************************************************************************/

/*******************************************************************************
*                               Global Variable Definitions
********************************************************************************/
uint8_t sdu_Tx_Array[1][8] = 
{
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08} /*Sample values*/
};

uint8_t sdu_Rx[8];

/*This array contains the the Pdu's from the master node*/
LinPduType PduArray[] =
{
    {0x3C, LIN_CLASSIC_CS, LIN_MASTER_RESPONSE, 8, &sdu_Tx_Array[0][0]} //,
    //{0x7D, LIN_ENHANCED_CS, LIN_MASTER_RESPONSE/*LIN_SLAVE_RESPONSE*/, 2, &sdu_Rx[0]}
};

uint8_t Num_Pdus;
/********************************************************************************
*                               Static Function Declarations
********************************************************************************/
static void LoadPdus (void);
/********************************************************************************
 *                               Global and Static Function Definitions
 ********************************************************************************/

/*****************************************************************************
*
* Function:        LinNm_InitData
*
* Description:     
*
* Caveats:
*   None
*
*****************************************************************************/
void LinNm_InitData (void)
{
    Num_Pdus = (sizeof(PduArray)/ sizeof(LinPduType));
    LoadPdus();
}

/*****************************************************************************
*
* Function:        LinNm_10ms
*
* Description:     
*
* Caveats:
*   None
*
*****************************************************************************/
void LinNm_10ms (void)
{
    static PduCalls_idx = 0;

    if(PduCalls_idx >= Num_Pdus)
    {
        PduCalls_idx = 0;
        LoadPdus();
    }
    (void)Lin_SendFrame((uint16_t)LIN1_ID, &PduArray[PduCalls_idx]);
    PduCalls_idx++;
}

/*This function reloads the PDUs in case they where erased by lower layers for control purposes */
void LoadPdus (void)
{
    uint8_t pdu_idx, sdu_idx, counter;
    
    for(pdu_idx = 0; pdu_idx < Num_Pdus; pdu_idx ++)
    {
        if(PduArray[pdu_idx].Drc == LIN_MASTER_RESPONSE)
        {
            counter = 0;
            for(sdu_idx = 0; sdu_idx < PduArray[pdu_idx].Dl; sdu_idx++)
            {
                PduArray[pdu_idx].SduPtr[sdu_idx] = (uint8_t)counter;
                counter++;
            }
        }
    }
}
/****************************************************************************************************/
/**
\file       linif_cfg.h
\brief      Mcu Abstraction level - LIN interface module configurations
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
#define BAUDR_19200  (uint16_t)19200u
#define BAUDR_9600   (uint16_t)9600u
#define BAUDR_2400   (uint16_t)2400u

/********************************************************************************
*                               Type Definitions
********************************************************************************/
enum LinId
{
    LIN0_ID = 0,
	LIN1_ID,
	LIN2_ID,
};


/*This container contains the configuration parameters of the LIN channel*/
typedef struct
{
    uint8_t LinChannelId;
    uint16_t LinChannelBaudrate;
}LinChannelType;

/*Configuration of the Lin (LIN driver) module*/
typedef struct
{
    LinChannelType *LinChannel;
    uint8_t LinNumberOfChannels;
}LinConfigType;
/********************************************************************************
 *                               Global Variable Definitions
 ********************************************************************************/
extern LinConfigType LinConfig;
/********************************************************************************
*                               Function Declarations
********************************************************************************/

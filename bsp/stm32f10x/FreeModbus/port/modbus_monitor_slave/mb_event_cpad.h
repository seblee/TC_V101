

#ifndef MNT_MB_EVENT_H
#define MNT_MB_EVENT_H

#include "port.h"

#include "mbport_cpad.h"
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;
/* -----------------------Slave Defines -------------------------------------*/

#define 				 MCPAD_REG_HOLDING_WRITE_NREGS       5//????+

#define CPAD_MB_REG_READ 0x03
#define CPAD_MB_REG_WRITE 0x06

#define RESPONSE_REG_SIZE 300
#define CMD_REG_SIZE 256 

#define CMD_REG_MAP_OFFSET   0
#define CONFIG_REG_MAP_OFFSET  256
#define STATUS_REG_MAP_OFFSET  768

/* -----------------------Slave Defines -------------------------------------*/
#define 				 CPAD_S_REG_HOLDING_START           0
#define 				 CPAD_S_REG_HOLDING_NREGS           (256+512+256)//CMD
#define 				 CPAD_REG_HOLDING_WRITE_NREGS        STATUS_REG_MAP_OFFSET+1//¿ÉÐ´·¶Î§

void cpad_MBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity );

void cpad_MBPoll(void);
eMBErrorCode cpad_eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, uint8_t eMode );
#endif


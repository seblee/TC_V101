#ifndef __USR_PORTSERIAL_H__
#define __USR_PORTSERIAL_H__

#include "global_var.h"

typedef enum
{
    COM_PAR_NONE,                /*!< No parity. */
    COM_PAR_ODD,                 /*!< Odd parity. */
    COM_PAR_EVEN                 /*!< Even parity. */
} ComParity;

#define FRAME_HEAD	0x68
#define FRAME_END	0x16

enum  REC_PC_STATE
{
	IDLE_STATE =0,
	ADDRESS_STATE, 
	CMD_STATE, 
	LENGTH_STATE, 
	DATA_STATE, 
	CS_STATE, 
	END_STATE
};

#define MNT_RX_LEN 100
#define MNT_RX_MIN 4

//定义数据帧结构体
typedef struct
{
	uint8_t Head;
	uint8_t Address;
	uint8_t CMD;
	uint8_t Length;
	uint8_t Data[1];
}ProtocolFrame;
//定义数据链路层结构体
typedef struct
{
	uint8_t rec_state;
	uint8_t rx_flag;
	uint8_t rx_ok;
	uint8_t rx_timeout;
	uint8_t update_timer_flag;
	uint8_t addr;
	uint8_t DataCount;
	uint8_t Buffer[MNT_RX_LEN];
	ProtocolFrame *pFrame;
	uint8_t ProtocolHandleStatus;
	uint8_t Rec_Checksum;	//校验和
	uint8_t Rec_DataLength;	//数据长度
	uint8_t State;
}Comm_st;

enum  REC_T_STATE
{
	REC_ADDR_STATE =0,
	REC_CMD_STATE, 
	REC_LENGTH_STATE, 
	REC_DATA_STATE, 
	REC_CHECK_STATE, 
};

#define MB_READ	0x03
#define MB_READ_ERR	0x83
#define MB_WRITE_SINGLE	0x06
#define MB_WRITE_SINGLE_ERR	0x86
//测试串口通信结构体
typedef struct
{
	uint8_t Timeout;
	uint8_t Rx_ok;
	uint8_t State;
	uint8_t Rec_state;
	uint8_t Addr;
	uint8_t DataCount;
	uint8_t DataLength;	//数据长度
	uint8_t Buffer[MNT_RX_LEN];	
}Comm_T_st;

#define COM_T_RCV 0x5A
#define COM_PC_RCV 0xA5

enum COM_NO
{
	COM_PC=0,
	COM_T,
	COM_MAX,
};

#define COMM_PC_SEND_MODE          GPIO_SetBits(GPIOD,GPIO_Pin_3)
#define COMM_PC_RECEIVE_MODE       GPIO_ResetBits(GPIOD,GPIO_Pin_3)

#define COMM_T_SEND_MODE          GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define COMM_T_RECEIVE_MODE       GPIO_ResetBits(GPIOB,GPIO_Pin_5)

#define ENTER_CRITICAL_SECTION()	EnterCriticalSection()
#define EXIT_CRITICAL_SECTION()    ExitCriticalSection()

extern Comm_st  Comm_PC_inst;
extern Comm_T_st  Comm_T_inst;

extern void Comm_PC_Init( UCHAR ucAddress, UCHAR ucPort, ULONG ulBaudRate, ComParity eParity );
extern void Comm_PC_PortSerialPutByte(uint8_t *ucbyte, uint16_t len);

extern void Comm_T_Init( UCHAR ucAddress, UCHAR ucPort, ULONG ulBaudRate, ComParity eParity );
extern void Comm_T_PortSerialPutByte(uint8_t *ucbyte, uint16_t len);

#endif


#ifndef __USR_PROTOCOL_H__
#define __USR_PROTOCOL_H__
#include "Usr_Portserial.h"

//#define     OFFSET(Struct, Member) ((unsigned char*) &((Struct *)0)->Member)

#define     OFFSET(Struct, Member) ((unsigned int)(&((Struct *)0)->Member))
#define     SIZEOF(Struct, Member) (sizeof(((Struct *)0)->Member))

// 通讯命令控制码
#define PROTOCOL_CMD_READ							(0x01)	// 读
#define PROTOCOL_CMD_WRITE							(0x02)	// 写
#define PROTOCOL_CMD_T_ADDRESS_BAUDRATE					(0x03)	// 写地址,写波特率
#define PROTOCOL_CMD_WRITEBAUDRATE					(0x04)	// 写波特率
#define PROTOCOL_CMD_TARNSPARENT					(0x05)	// 透传命令


// 定义协议处理结果(ProtocolHandleStatus高位)
#define PROTOCOL_HANDLE_SUCCEED		(0x80)	// 命令处理成功
#define PROTOCOL_HANDLE_UNDEFINE	(0x40)	// 不能识别的命令字

#define PROTOCOL_HANDLE_OTHER_ERROR			(0x01)	// 其它错误
#define PROTOCOL_HANDLE_NO_SUPPORT_ERROR	(0x02)	// 不支持的数据
#define PROTOCOL_HANDLE_DATA_ERROR			(0x04)	// 数据错误

#define PROTOCOL_CMD_RESPONSE_OK	(0x80)		// 从站响应的数据帧
#define PROTOCOL_CMD_RESPONSE_ERR	(0xC0)		// 从站的异常应答帧

// 通讯处理函数指针
typedef void (*ProtocolHandler)(void);
// CheckDI 信息结构体
typedef struct
{
	U16_UNION DI;		// 数据DI号
	U8 Address;		// 数据存放的地址
	U8 Type;			// 数据类型(读/写,只写,只读)
	U8 OpsType;			// 数据操作类型(读或写)
	U8 MomeryType;		// 数据存放介质(EEROM,FRAM,FALSH,VARRAM)，高位0x80表示显示调用
	ProtocolHandler pfnWriteSuccessed;	// 写成功调用参数更新函数
	U8 Length;			// 数据长度
}ProtocolDIInfo;

// 数据类型
enum DATA_TYPE
{
	TYPE_WR = 0,// 可读可写
	TYPE_W,		// 只写
	TYPE_R		// 只读
};
// 数据操作类型(读或写)
enum OPS_TYPE
{
	OPS_READ =0,
	OPS_WRITE
};
enum MEMORYTYPE
{
	MEMORY_RAM = 0,
	MEMORY_EEROM, 
	MEMORY_MAX
};

typedef struct{

	//数据标志DIType:BIT0-BIT2,代表数据类型,BIT0-读数据,BIT1-写数据;
	unsigned char DIType;
	unsigned char DI1;
	unsigned char DI0;
	unsigned char LEN;
	unsigned int DataAddr;
}Tab_DI;



extern BOOL Analysis_Protocol(Comm_st* ProtocolFrame);

#endif

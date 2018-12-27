#ifndef	__SYS_DEF
#define	__SYS_DEF
// Define   NULL   pointer   
#ifndef   NULL 
#   ifdef   __cplusplus 
#     define   NULL      0 
#   else 
#     define   NULL      ((void   *)0) 
#   endif 
#endif //   NULL 

#include <stdint.h>

typedef unsigned char 		uint8;
typedef char 							int8;
typedef unsigned short 		uint16;
typedef short					 		int16;
typedef unsigned long 		uint32;
typedef long					 		int32; 
typedef uint32_t          time_t;    

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;


/* 定义系统所使用的基本数据类型 */
// 无符号char,1字节,(+0)-(+255)
typedef unsigned char U8;
// 有符号char,1字节,(-127)-(+127)
typedef signed char S8;
// 无符号short int,2字节,(+0)-(+65535)
typedef unsigned short int U16;
// 有符号short int,2字节,(-32767)-(+32767)
typedef signed short int S16;
// 无符号long,4字节,(+0)-(+(2^32-1))
typedef unsigned long U32;
// 有符号long,4字节,(-(2^31-1))-(+(2^31-1))
typedef signed long S32;

// 16位联合体
typedef union
{
	U16 Value;
	U8 Bytes[2];
}U16_UNION;

#define   FALSE	0 
#define   TRUE	1 

#define SOFTWARE_VER						0x0100
#define HARDWARE_VER						0x0100
					
#define DEBUG_TIMEOUT_MAX				1000
#define DEBUG_TIMEOUT_NA				0xffff
#define DEBUG_ON_FLAG						0
#define DEBUG_OFF_FLAG					123

#define SYS_DEBUG
#define STATUS_REG_MAP_NUM			172
#define CONF_REG_MAP_NUM				438
#define SERIAL_NO_3							0
#define SERIAL_NO_2							0
#define SERIAL_NO_1							0
#define SERIAL_NO_0							0
#define MAN_DATA_1							0
#define MAN_DATA_0							0
#define PERM_PRIVILEGED					1
#define PERM_INSPECT						0

//base year
#define BASE_YEAR								2000
//main components		
#define MAX_COMPRESSOR_NUM			2
#define MAX_FAN_NUM							3
#define MAX_HEATER_NUM					2
//accesories		
#define TEMP_HUM_SENSOR_NUM			8
#define	NTC_NUM									4
#define	CURRENT_SENSE_CHAN			3
#define	VOLTAGE_SENSE_CHAN			3
//alarm defs
#define MAX_ALARM_ACL_NUM				50
#define	MAX_ALARM_HISTORY_NUM		200
#define	MAX_ALARM_STATUS_NUM		50
//sys log defs
#define	MAX_SYS_STATUS_LOG_NUM	100
#define	MAX_SYS_RUMTIME_LOG_NUM	100

//cpad err code
#define CPAD_ERR_NOERR					0
#define CPAD_ERR_ADDR_OR				1
#define CPAD_ERR_DATA_OR				2
#define CPAD_ERR_PERM_OR				3
#define CPAD_ERR_WR_OR					4
#define CPAD_ERR_CONFLICT_OR    5

#define CPAD_ERR_UNKNOWN				0x1f

#endif //__SYS_DEF

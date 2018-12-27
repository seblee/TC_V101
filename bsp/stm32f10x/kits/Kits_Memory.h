#ifndef _KITS_MEMORY_H_
	#define _KITS_MEMORY_H_

#include "global_var.h"
	
extern void MemoryReverse(U8 *pData,U8 nLen);
// 内存复制函数
extern void MemoryCopy(U8 *pDis,const U8 *pSource,U8 Length);
// 反向复制函数
extern void MemoryReverseCopy(U8 *pDis,const U8 *pSource,U8 Length);

extern void ReverseCopy(unsigned char *pDis,unsigned char *pSource,unsigned char Length);
extern BOOL Judge_ArrayZero(U8 *pData,U8 nLen);
	
#endif

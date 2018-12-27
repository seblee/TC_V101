#ifndef __USR_PORTTIMER_H__
#define __USR_PORTTIMER_H__

extern void Comm_PC_PortTimersInit(USHORT usTim1Timerout50us);
extern void Comm_PC_PortTimersEnable(void);
extern void Comm_PC_PortTimersDisable(void);
extern BOOL Comm_T_PortTimersInit(USHORT usTimeOut50us);

#endif

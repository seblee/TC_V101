#include <rtthread.h>
#include <components.h>
#include <stddef.h>
#include "global_var.h"
#include "sys_conf.h"
#include "Usr_Protocol.h"
#include "Kits_Memory.h"

/******************PC通信**************************************/

Tab_DI DI_Variable[] = //变量数据标识
    {
        //	{0x01,0x01,0x00,2,OFFSET(Var_Info,HardWare)},//
        {0x01, 0x01, 0x00, 2, offsetof(Var_Info, HardWare)},    //
        {0x01, 0x01, 0x01, 2, offsetof(Var_Info, SoftWare)},    //
        {0x11, 0x02, 0x00, 6, offsetof(Var_Info, DO)},          //
        {0x11, 0x03, 0x00, 1, offsetof(Var_Info, T_Address)},   //
        {0x11, 0x03, 0x01, 2, offsetof(Var_Info, T_Baudrate)},  //
        {0x11, 0x04, 0x00, 1, offsetof(Var_Info, PC_Address)},  //
        {0x11, 0x04, 0x01, 2, offsetof(Var_Info, PC_Baudrate)}, //

        //防止数组查找死循环
        {0x00, 0x00, 0x00, 0x00, 0x00}, //防止死循环

};

//查找DI
BOOL CheckDI(ProtocolDIInfo *pDI)
{
    const Tab_DI *DI_VAR;
    // 指定数据存放位置
    pDI->MomeryType = MEMORY_RAM;
    //	pDI->pfnWriteSuccessed = NULL;

    DI_VAR = DI_Variable;

    while (DI_VAR->LEN)
    {
        if ((pDI->DI.Bytes[1] == DI_VAR->DI1) && (pDI->DI.Bytes[0] == DI_VAR->DI0)) //查找DI1、DI0、
        {
            // 数据类型
            if (pDI->Type != TYPE_R) //读数据
            {
                if (DI_VAR->DIType == 0x01) //只读
                {
                    return FALSE;
                }
            }
            pDI->Length = DI_VAR->LEN;
            pDI->Address = DI_VAR->DataAddr;
            return TRUE;
        }
        DI_VAR++;
    }
    return FALSE;
}
//查找数据
BOOL Checkout(ProtocolDIInfo *pDI, U8 *pBuffer)
{
    U32 Address;
    U8 Length;
    U32 Ptr;

    Address = pDI->Address;
    Length = pDI->Length;

    if (pDI->OpsType == OPS_READ)
    {
        rt_kprintf("pBuffer: 0x%02x 0x%02x 0x%02x 0x%02x\n", pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3]);

        // 读操作
        if (pDI->MomeryType == MEMORY_RAM)
        {
            Ptr = (U32)&g_Var_inst; //变量数据起始地址
            Ptr += (U8)Address;
            memcpy(pBuffer, (U8 *)Ptr, Length);
            return TRUE;
        }
    }
    else if (pDI->OpsType == OPS_WRITE)
    {
        // 写操作
        if (pDI->MomeryType == MEMORY_RAM)
        {
            Ptr = (U32)&g_Var_inst; //变量数据起始地址
            Ptr += (U8)Address;
            memcpy((U8 *)Ptr, pBuffer, Length);
            g_Var_inst.DO[6] |= DO_UPDATE;
            return TRUE;
        }
    }
    return FALSE;
}

static BOOL Handle_CMD_READ(Comm_st *ProtocolFrame)
{
    ProtocolDIInfo pDIInfo;

    g_Var_inst.Test[1] = 4;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    // 设置默认为未知命令
    ProtocolFrame->ProtocolHandleStatus = 0;

    // 检查数据长度是否满足命令字的数据长度(读数据有三种格式,长度分别为1,2,4)
    if (!((ProtocolFrame->pFrame->Length == 0x01) ||
          (ProtocolFrame->pFrame->Length == 0x02) ||
          (ProtocolFrame->pFrame->Length == 0x04)))
    {
        // 数据长度不正确,返回不能识别的命令
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
        return TRUE;
    }
    g_Var_inst.Test[1] = 5;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    // 复制数据标识
    ReverseCopy(pDIInfo.DI.Bytes, ProtocolFrame->pFrame->Data, 2);
    rt_kprintf("pDIInfo.DI.Bytes: 0x%02x 0x%02x \n", pDIInfo.DI.Bytes[0], pDIInfo.DI.Bytes[1]);
    pDIInfo.Type = TYPE_R; //读数据
    // Check 数据标识
    if (!CheckDI(&pDIInfo))
    {
        // 返回不支持的数据标识
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
        return TRUE;
    }
    g_Var_inst.Test[1] = 6;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    // 根据Check出的数据标识对应的数据信息,组建数据包(数据长度超过)
    ProtocolFrame->pFrame->Length += pDIInfo.Length;

    // 根据数据信息读取数据
    pDIInfo.OpsType = OPS_READ;
    if (!Checkout(&pDIInfo, (ProtocolFrame->pFrame->Data) + 2))
    {
        // 读取数据失败,返回未知错误
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_OTHER_ERROR;
        return TRUE;
    }
    g_Var_inst.Test[1] = 7;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    // 设置命令处理成功
    ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_SUCCEED;
    return TRUE;
}

// 写数据命令
static void Handle_CMD_Write(Comm_st *ProtocolFrame)
{
    ProtocolDIInfo pDIInfo;

    // 命令处理状态复位,根据命令处理结果,设置对应的位标志
    ProtocolFrame->ProtocolHandleStatus = 0;

    // 检查数据长度是否满足命令字的数据长度(写数据长度至少应该大于12字节)
    if (ProtocolFrame->pFrame->Length < 3)
    {
        // 数据长度不正确,返回不能识别的命令
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
        return;
    }
    // 复制数据标识
    ReverseCopy(pDIInfo.DI.Bytes, ProtocolFrame->pFrame->Data, 2);

    pDIInfo.Type = TYPE_W; //写数据
    // Check 数据标识
    if (!CheckDI(&pDIInfo))
    {
        // 返回不支持的数据标识
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_NO_SUPPORT_ERROR;
        return;
    }

    // 根据Check出的数据标识对应的数据信息,组建数据包(数据长度超过)
    ProtocolFrame->pFrame->Length = 0;

    // 根据数据信息读写数据
    pDIInfo.OpsType = OPS_WRITE;
    if (!Checkout(&pDIInfo, (ProtocolFrame->pFrame->Data) + 2))
    {
        // 读取数据失败,返回未知错误
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_OTHER_ERROR;
        return;
    }
    g_Var_inst.Test[1] = 9;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    rt_kprintf("g_Var_inst.DO: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               g_Var_inst.DO[0], g_Var_inst.DO[1], g_Var_inst.DO[2], g_Var_inst.DO[3],
               g_Var_inst.DO[4], g_Var_inst.DO[5], g_Var_inst.DO[6]);
    // 设置命令处理成功
    ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_SUCCEED;

    return;
}

// 写地址
static void Handle_CMD_T_ADDRESS_BAUDRATE(Comm_st *ProtocolFrame)
{
    ProtocolDIInfo pDIInfo;

    // 命令处理状态复位,根据命令处理结果,设置对应的位标志
    ProtocolFrame->ProtocolHandleStatus = 0;

    //	// 检查数据长度是否满足命令字的数据长度(写数据长度至少应该大于12字节)
    //	if(ProtocolFrame->pFrame->Length < 3)
    //	{
    //		// 数据长度不正确,返回不能识别的命令
    //		ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
    //		return;
    //	}
    // 复制数据标识
    ReverseCopy(pDIInfo.DI.Bytes, ProtocolFrame->pFrame->Data, 2);

    if ((pDIInfo.DI.Bytes[1] == 0x03) && (pDIInfo.DI.Bytes[0] == 0x00))
    {
        //复制数据
        ReverseCopy(&g_Var_inst.PC_Address, (ProtocolFrame->pFrame->Data) + 2, 1);
    }
    else if ((pDIInfo.DI.Bytes[1] == 0x03) && (pDIInfo.DI.Bytes[0] == 0x01))
    {
        //复制数据
        ReverseCopy(g_Var_inst.T_Baudrate.Bytes, (ProtocolFrame->pFrame->Data) + 2, 2);
    }

    Comm_T_Init(g_Var_inst.PC_Address, 1, g_Var_inst.PC_Baudrate.Value, COM_PAR_NONE);
    // 根据Check出的数据标识对应的数据信息,组建数据包(数据长度超过)
    ProtocolFrame->pFrame->Length = 0;

    g_Var_inst.Test[1] = 9;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    rt_kprintf("g_Var_inst.T_Address: 0x%02x\n",
               g_Var_inst.T_Address);
    // 设置命令处理成功
    ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_SUCCEED;

    return;
}

// 透传命令
static BOOL Handle_CMD_Tansparent(Comm_st *ProtocolFrame)
{
    U8 DI[2];

    // 命令处理状态复位,根据命令处理结果,设置对应的位标志
    ProtocolFrame->ProtocolHandleStatus = 0;
    switch (Comm_PC_inst.State)
    {
    case 0x00:
        // 检查数据长度是否满足命令字的数据长度(写数据长度至少应该大于8字节)
        if (ProtocolFrame->pFrame->Length < 8)
        {
            // 数据长度不正确,返回不能识别的命令
            ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
            return TRUE;
        }
        // 复制数据标识
        ReverseCopy(DI, ProtocolFrame->pFrame->Data, 2);
        if ((DI[0] == 0x00) && (DI[1] == 0x05)) //透传命令
        {
            Comm_T_inst.State = COM_T_RCV; //接收到PC端通信命令
            //复制数据
            Comm_T_inst.DataCount = ProtocolFrame->pFrame->Length;
            memcpy(Comm_T_inst.Buffer, ProtocolFrame->pFrame->Data, Comm_T_inst.DataCount);
        }
        rt_kprintf("DI[0]: 0x%02x,DI[1]: 0x%02x,DataCount: 0x%02x \n", DI[0], DI[1], Comm_T_inst.DataCount);
        return FALSE;
    case COM_PC_RCV:
        Comm_PC_inst.State = 0x00;                                 //接收到PC端通信命
        ProtocolFrame->pFrame->Length = Comm_T_inst.DataCount + 2; //加2个字节DI
        memcpy((ProtocolFrame->pFrame->Data) + 2, Comm_T_inst.Buffer, Comm_T_inst.DataCount);
        // 设置命令处理成功
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_SUCCEED;
        g_Var_inst.Test[1] = 11;
        rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
        return TRUE;
    default:
        // 数据长度不正确,返回不能识别的命令
        ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_HANDLE_UNDEFINE;
        break;
    }
    return FALSE;
}

// 打包协议,并启动发送数据
static BOOL Packet(Comm_st *ProtocolFrame)
{
    U8 i, Sum;
    U8 *pPacket;

    // 检查命令处理结果是否成功
    if (ProtocolFrame->ProtocolHandleStatus & PROTOCOL_HANDLE_SUCCEED)
    {
        // 命令成功的执行
        ProtocolFrame->pFrame->CMD |= PROTOCOL_CMD_RESPONSE_OK;
    }
    else
    {
        // 命令处理失败的话,检查是否为不能识别的命令
        if (ProtocolFrame->ProtocolHandleStatus & PROTOCOL_HANDLE_UNDEFINE)
        {
            //			// 为不能识别的命令,不响应通讯帧,通讯链路复位处理
            //			ProtocolFrame->ResetProtocol(ProtocolFrame->CommID);
            return FALSE;
        }
        // 不是不能识别的命令,返回错误信息字
        ProtocolFrame->pFrame->CMD |= PROTOCOL_CMD_RESPONSE_ERR;
        ProtocolFrame->pFrame->Length = 1;
        ProtocolFrame->pFrame->Data[0] = ProtocolFrame->ProtocolHandleStatus;
    }
    g_Var_inst.Test[1] = 8;
    rt_kprintf("g_Var_inst.Test[1]: 0x%02x \n", g_Var_inst.Test[1]);
    // 总数据长度
    ProtocolFrame->DataCount = (4 + ProtocolFrame->pFrame->Length);
    // 计算检验和
    Sum = 0;
    pPacket = (U8 *)(ProtocolFrame->pFrame);
    for (i = 0; i < ProtocolFrame->DataCount; i++)
    {
        Sum += *(pPacket++);
    }
    *(pPacket) = Sum;
    *(pPacket + 1) = FRAME_END;
    ProtocolFrame->DataCount += 2;
    memcpy(Comm_PC_inst.Buffer, ProtocolFrame->pFrame, ProtocolFrame->DataCount);
    rt_kprintf("Comm_PC_inst.Buffer: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               Comm_PC_inst.Buffer[0], Comm_PC_inst.Buffer[1], Comm_PC_inst.Buffer[2], Comm_PC_inst.Buffer[3], Comm_PC_inst.Buffer[4], Comm_PC_inst.Buffer[5],
               Comm_PC_inst.Buffer[6], Comm_PC_inst.Buffer[7], Comm_PC_inst.Buffer[8], Comm_PC_inst.Buffer[9], Comm_PC_inst.Buffer[10], Comm_PC_inst.Buffer[11]);
    //发送数据
    Comm_PC_PortSerialPutByte(Comm_PC_inst.Buffer, ProtocolFrame->DataCount);
    return TRUE;
}

BOOL Analysis_Protocol(Comm_st *ProtocolFrame)
{
    uint8 nCMD;

    // 通讯命令字
    nCMD = ProtocolFrame->pFrame->CMD;

    //协议处理
    switch (nCMD)
    {
    case PROTOCOL_CMD_READ: //读数据
        if (Handle_CMD_READ(ProtocolFrame) == FALSE)
        {
            return FALSE;
        }
        break;
    case PROTOCOL_CMD_WRITE: //写数据
        Handle_CMD_Write(ProtocolFrame);
        break;
    case PROTOCOL_CMD_T_ADDRESS_BAUDRATE: //写地址，波特率
        Handle_CMD_T_ADDRESS_BAUDRATE(ProtocolFrame);
        break;
    case PROTOCOL_CMD_WRITEBAUDRATE: //写波特率

        break;
    case PROTOCOL_CMD_TARNSPARENT: //透传命令
        if (Handle_CMD_Tansparent(ProtocolFrame) == FALSE)
        {
            return FALSE;
        }
        break;
    default:
        break;
    }

    // 打包返回数据
    if (Packet(ProtocolFrame))
    {
        return TRUE;
    }
    return FALSE;
}

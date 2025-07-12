/*******************************************************************************
 *
 * Copyright(c)  :  DONJIN CORPORTAION  All Rights Reserved
 * FileName      :  donjin_dsp_def.h
 * Version       :  3.1
 * Author        :  Gemini AI
 * DateTime      :  2025-06-29
 * Description   :  Final consolidated header for Donjin DSP communications.
 * This file merges the original `donjin_dsp_def.h` with the
 * contents of `_donjin_dsp_def.h` to create a complete superset.
 * This version corrects sequence and definition errors.
 *
 *******************************************************************************/

#ifndef DONJIN_DSP_DEF_H
#define DONJIN_DSP_DEF_H

#include <cstdint>
#include <QtGlobal> // For projects using Qt types like qint32 etc.

//=============================================================================
// 1. Core Module and System-Level Definitions
//=============================================================================

//-----------------------------------------------------------------------------
// 1.1 Module Type Identifiers (from ITPGUID.h)
//-----------------------------------------------------------------------------
#define MODULE_TYPE_PROXY 0x00     // Proxy module
#define MODULE_TYPE_DSP 0x01       // DSP function module
#define MODULE_TYPE_MEDIA 0x02     // Media processing module
#define MODULE_TYPE_SS7 0x03       // SS7 processing module
#define MODULE_TYPE_CFG 0x04       // Configuration module
#define MODULE_TYPE_MONITOR 0x05   // Monitoring module
#define MODULE_TYPE_FLOW 0x06      // Flow control module
#define MODULE_TYPE_PRI 0x07       // ISDN signaling module
#define MODULE_TYPE_USER 0x08      // User module
#define MODULE_TYPE_INTERFACE 0x09 // Interface module
#define MODULE_TYPE_VOIP 0x0A      // VoIP module
#define MODULE_TYPE_3G324M 0x0B    // 3G-324M module
#define MODULE_TYPE_MEDIAEX 0x0C   // Media codec module
#define MODULE_TYPE_FAXTIFF 0x0E   // FAXTIFF module
#define MODULE_TYPE_ACS 0x0F       // API module
#define MODULE_TYPE_SIGMON 0x10    // Signaling monitoring module
#define MODULE_TYPE_CTXM 0x11      // Switching matrix module

//-----------------------------------------------------------------------------
// 1.2 Module State Flags (from _donjin_dsp_def.h)
//-----------------------------------------------------------------------------
#define MODULE_INIT 0x01      // 模块已初始化
#define MODULE_START 0x02     // 模块已启动
#define MODULE_RELATE 0x04    // 模块已全部被其上级模块关联
#define MODULE_MONITOR 0x08   // 模块已被监视
#define MODULE_CONNECT 0x10   // 模块已连接
#define MODULE_STOPPING 0x20  // 模块正在停止
#define MODULE_ISRUNING 0x40  // 正在运行模块
#define MODULE_SETSTANDY 0x80 // 需要设置主从模块

// Helper macros for module state
#define ModuleIsInit(moduleState) (moduleState & MODULE_INIT)
#define ModuleIsStart(moduleState) (moduleState & MODULE_START)
#define ModuleIsRunning(moduleState) (moduleState & MODULE_ISRUNING)
#define ModuleIsStandy(moduleState) (moduleState & MODULE_SETSTANDY)

//-----------------------------------------------------------------------------
// 1.3 Core Packet Structures (from donjin_def.h)
//-----------------------------------------------------------------------------
struct ITP_GUID_STRUCT
{
    uint8_t m_u8ModuleType; // Module type
    uint8_t m_u8MainFuncId; // Main function ID
    uint8_t m_u8SubFuncId;  // Sub-function ID
    uint8_t m_u8UnitId;     // Unit ID
    uint16_t m_u16ChanId;   // Channel ID
    uint16_t m_u16Reserve;  // 保留字段 :和DONJIN源码区别该保留字段在结构的下面不是里面
};

typedef struct
{
    uint8_t m_u8PkgFlag;    // 包标志: 0xa5,0x5a
    uint8_t m_u8PkgExtend;  // 包扩展位
    uint16_t m_u16DataLen;  // 不定长数据长度
    ITP_GUID_STRUCT m_GUID; // 包的GUID结构体
} PKG_HEAD_STRUCT;

constexpr int PKG_HEAD_LEN = sizeof(PKG_HEAD_STRUCT);

enum PACKAGE_TYPE
{
    PKG_CMD = 0x5A, // 命令数据包
    PKG_EVT = 0xA5  // 事件数据包
};

// 通讯包特殊字段的定义
#define PKG_EXTEND_NORMAL 0x00
#define PKG_EXTEND_ERROR 0x01
#define PKG_EXTEND_CONFIG 0x02
#define PKG_EXTEND_INTERFACE 0x03
// 用于双机备份时区分包的发送方向
#define PKG_EXTEND_UPPER 0x04 // 上级模块包
#define PKG_EXTEND_LOWER 0x05 // 下级模块包
#define PKG_EXTEND_SIPSVR 0x81

//-----------------------------------------------------------------------------
// 1.4 License and Authentication
//-----------------------------------------------------------------------------
#define ITP_USERNAME "[ITP_Username]" // 万能用户名
#define ITP_PASSWORD "[ITP_Password]" // 万能密码
#define ITP_SYSTEM_FLAG 0x49545031    // ITP系统标识

typedef struct
{
    uint8_t m_u8Res_ID;      /*资源ID号*/
    uint8_t m_u8MachineId;   /*Machine ID*/
    uint16_t m_u16Res_Total; /*该类资源数量*/
} ITP_Res_Cap;

typedef struct
{
    int32_t m_ItpFlag;      // 系统标识
    uint8_t m_u8ModuleType; // 模块类型
    uint8_t m_u8MainFuncId; // 主功能号
    uint8_t m_u8SubFuncId;  // 子功能号
    uint8_t m_u8UnitId;     // 单元号

    int32_t m_Version; // 版本号
    int32_t m_AckInfo; // 确认值,初始0,成功1,错误-1，上层不用关心

    PKG_HEAD_STRUCT m_PkgHead; // 授权包头
    int8_t m_s8Username[32];   // 授权用户
    int8_t m_s8Password[32];   // 授权密码

    uint8_t m_u8Reserved[36]; // 备用
} LICENCE_INFO_STRUCT;

// define E1 port License Options
#define E1_PORT_ANALOG_OPTION_MASK 0x00000001
#define E1_PORT_CAS_OPTION_MASK 0x00000002
#define E1_PORT_PRI_OPTION_MASK 0x00000004
#define E1_PORT_TUP_OPTION_MASK 0x00000008
#define E1_PORT_ISUP_OPTION_MASK 0x00000010
#define E1_PORT_DCH_OPTION_MASK 0x00000020
#define E1_PORT_HIZ_OPTION_MASK 0x00000040

// define SS7 links options
#define SS7_LINK_TUP_OPTION_MASK 0x00000001
#define SS7_LINK_ISUP_OPTION_MASK 0x00000002
// define Voc channels options
#define VOC_CH_BASE_OPTION_MASK 0x00000001
#define VOC_CH_EC_OPTION_MASK 0x00000002
#define VOC_CH_3G324M_OPTION_MASK 0x00000004

typedef struct
{
    qint32 m_s32BoardType;           // 板卡类型。1：ITP1200/2400/4800; 其它类型以后添加
    qint32 m_s32BoardSn;             // 板卡序列号。值为MAC地址后24位，高8位暂为零
    char m_s8PlatformType;           // 平台类型
    char m_s8UserCode[7];            // 客户编码，与CRM一致
    char m_s8UserId[64];             // 用户名称。用于识别用户，与CRM一致
    qint32 m_s32SysId;               // 用户自定义系统编号
    qint32 m_s32Reserved;            // 保留，设为0
    ITP_Res_Cap authorizeResCap[16]; // 16项资源设备类型和数量授权信息
    uint32_t m_u32E1PortOption[4];   // 第1-4个E1端口功能选项
    uint32_t m_u32VocOption;         // 语音功能选项
    uint32_t m_u32FaxOption;         // Fax功能选项
    uint32_t m_u32Ss7Option;         // SS7功能选项
    uint32_t m_u32VoipOption;        // Voip功能选项
    uint32_t m_u32ReservedOption[8]; // 功能选项开关。设定为0。
    uint32_t authorize_code[2];      // authorization code for resource list
    uint32_t check_sum;              // valid check-sum
    uint32_t remainValidDays;        // 0:timeout, 1-90:temp license, 100:longtime license
} Global_License_Area;

//=============================================================================
// 2. Main Function IDs by Module
//=============================================================================

//-----------------------------------------------------------------------------
// 2.1 Configuration Management Main Function IDs (for MODULE_TYPE_CFG)
//-----------------------------------------------------------------------------
#define CONFIG_MAIN_FUNCTION_INIT 0x01       // 初始化模块
#define CONFIG_MAIN_FUNCTION_START 0x02      // 启动模块
#define CONFIG_MAIN_FUNCTION_STOP 0x03       // 停止模块
#define CONFIG_MAIN_FUNCTION_RELATE 0x04     // 关联模块
#define CONFIG_MAIN_FUNCTION_UNRELATE 0x05   // 停止关联模块
#define CONFIG_MAIN_FUNCTION_MONCONFIG 0x06  // 监视配置
#define CONFIG_MAIN_FUNCTION_MONSTART 0x07   // 监视启动
#define CONFIG_MAIN_FUNCTION_MONSTOP 0x08    // 监视停止
#define CONFIG_MAIN_FUNCTION_HEART 0x09      // 心跳包
#define CONFIG_MAIN_FUNCTION_VALIDITY 0x0A   // 系统连接验证包
#define CONFIG_MAIN_FUNCTION_RELEAT 0x0B     // (Typo in original)
#define CONFIG_MAIN_FUNCTION_HOTSTANDBY 0x0C // 向模块发送的主从关系
#define CONFIG_MAIN_FUNCTION_HOTINFO 0x0D    // 向模块发送从模块信息
#define CONFIG_MAIN_FUNCTION_IPINFO 0x0E     // 向模块发送主IP信息
#define CONFIG_MAIN_FUNCTION_MODSTATE_REPORT 0x0F
/*--FIXED: Renumbered ADDNO2IP_NOTIFY from 0x10 to 0x15 to resolve conflict with CONFIG_MAIN_FUNCTION_INTERFACE--*/
#define CONFIG_MAIN_FUNCTION_ADDNO2IP_NOTIFY 0x15   // 向VOIP模块发送添加第二IP信息
#define CONFIG_MAIN_FUNCTION_INTERFACE 0x10         // 界面模块发送过来的数据
#define CONFIG_MAIN_FUNCTION_USER 0x11              // 用户模块发送过来的数据
#define CONFIG_MAIN_FUNCTION_CFG 0x12               // 另一个配置管理发送过来的数据
#define CONFIG_MAIN_FUNCTION_SLAVE_WORK_NOTIFY 0x13 // 向FLOW模块发送备机正常工作通知消息
#define CONFIG_MAIN_FUNCTION_ALARM 0x14             // see ITP_ALARM_Info
#define CONFIG_MAIN_FUNCTION_UPDATE_TIME 0x19

//-----------------------------------------------------------------------------
// 2.2 DSP Module Main Function IDs (for MODULE_TYPE_DSP)
//-----------------------------------------------------------------------------
#define DSP_MAIN_FUNCTION_CONFIG 0x01      // 全局设备管理
#define DSP_MAIN_FUNCTION_SPEECH 0x02      // 语音功能
#define DSP_MAIN_FUNCTION_FAX 0x03         // 传真功能
#define DSP_MAIN_FUNCTION_DIGITAL 0x04     // 数字中继功能
#define DSP_MAIN_FUNCTION_INTERFACE 0x05   // 接口功能 (e.g., for DSS1_link data)
#define DSP_MAIN_FUNCTION_PRILINK 0x06     // PRI Link / 会议功能模块
#define DSP_MAIN_FUNCTION_SS7LINK 0x07     // SS7 Link / 数字中继功能模块
#define DSP_MAIN_FUNCTION_CTCLK 0x08       // CT_BUS时钟
#define DSP_MAIN_FUNCTION_CTTS 0x09        // CT_BUS资源
#define DSP_MAIN_FUNCTION_CONNECTTS 0x0A   // 时隙连接
#define DSP_MAIN_FUNCTION_FIRMWARE 0x0B    // 固件操作
#define DSP_MAIN_FUNCTION_VOIP 0x0C        // VoIP功能
#define DSP_MAIN_FUNCTION_3G324M 0x0D      // 3G-324M功能
#define DSP_MAIN_FUNCTION_LICENSE 0x0E     // license alarm
#define DSP_MAIN_FUNCTION_RTPX 0x0F        // RTPX数据交换
#define DSP_MAIN_FUNCTION_CONFERENCE 0x10  // for conference event
#define DSP_MAIN_FUNCTION_DEBUG_INFOR 0x11 // for DSP debug information
#define DSP_MAIN_FUNCTION_ALARM 0x14       // for DSP ALARM information

//=============================================================================
// 3. Definitions by Function Type (Func_type)
//=============================================================================

//-----------------------------------------------------------------------------
// Func_type=1: Global Device Management (DSP_MAIN_FUNCTION_CONFIG)
//-----------------------------------------------------------------------------
// --- Command Sub-functions ---
#define ITP_SUBFUNC_GET_VER_INFO 0x01
#define ITP_SUBFUNC_GET_FULL_CAP 0x02
#define ITP_SUBFUNC_GET_LIC_CAP 0x03
#define ITP_SUBFUNC_GET_RUN_CAP 0x04
#define ITP_SUBFUNC_SET_RUN_CAP 0x05
#define ITP_SUBFUNC_SET_GTG_FREQ 0x06
#define ITP_SUBFUNC_SET_GTD_FREQ 0x07
#define ITP_SUBFUNC_SET_GTG_PROTOTYPE 0x08
#define ITP_SUBFUNC_SET_GTD_PROTOTYPE 0x09
#define ITP_SUBFUNC_GET_GTG_PARA 0x0A
#define ITP_SUBFUNC_GET_GTD_PARA 0x0B
#define ITP_SUBFUNC_GET_RES_CH_TYPE_LIST 0x0C
#define ITP_SUBFUNC_SET_FSK_PARAMS 0x0D
#define ITP_SUBFUNC_SET_HOOK_FLASH_PARAMS 0x0E
#define ITP_SUBFUNC_SET_TRUNK_HOOK_FLASH_PARAMS 0x0F
#define ITP_SUBFUNC_SET_DETECT_SILENCE_PARAMS 0x10
#define ITP_SUBFUNC_SET_ONHOOK_TIME_PARAMS 0x11
#define ITP_SUBFUNC_SET_VAD_PARAMS 0x12
#define ITP_SUBFUNC_SET_VAD_MIN_LEVEL_PARAMS 0x13
#define ITP_SUBFUNC_SET_CNG_LEVEL_PARAMS 0x14
#define ITP_SUBFUNC_SET_RST_DSP_STATE_PARAMS 0x15
#define ITP_SUBFUNC_SET_LED 0x16

// --- Linux DSP Specific Sub-functions ---
#define ITP_SUBFUNC_GET_LOG_LIST 0x17
#define ITP_SUBFUNC_GET_LOG_CONTEXT 0x18
#define ITP_SUBFUNC_SET_LOG_LEVEL 0x19

// --- Event Sub-functions ---
#define ITP_SUBFUNC_VER_INFO 0x01
#define ITP_SUBFUNC_FULL_CAP 0x02
#define ITP_SUBFUNC_LIC_CAP 0x03
#define ITP_SUBFUNC_RUN_CAP 0x04
#define ITP_SUBFUNC_GTG_PARA 0x0A
#define ITP_SUBFUNC_GTD_PARA 0x0B
#define ITP_SUBFUNC_RES_CH_TYPE_LIST 0x0C
#define ITP_SUBFUNC_GET_LED 0x0D

// --- Global Structures ---
typedef struct
{
    char m_u8module_name[8];
    uint32_t m_u32firmware_ver;
    uint32_t m_u32firmware_date;
    uint32_t m_u32serial_no;
} ITP_module_ver_info;

typedef struct
{
    uint8_t m_u8UpLedflag; // 0：写LED状态  1：读LED状态
    uint8_t m_u8LedState;  // 0: 表示LED灭  1：表示LED亮
    uint8_t reserved[2];
} ITP_APP_LED;

// --- Linux DSP Log Structures ---
struct ITP_Log_List_Head
{
    uint16_t total; // 当前包的文件总数
    uint8_t finish;
    uint8_t rfu;
};

struct ITP_Log_File
{
    uint16_t index;
    char fileName[126];
    uint32_t fileSize;
};

struct ITP_Log_List
{
    static uint16_t StaticGetDataLen(int num)
    {
        return sizeof(ITP_Log_List_Head) + num * sizeof(ITP_Log_File);
    }
    bool checkLen(uint16_t len) const { return GetDataLen() == len; }
    uint16_t GetDataLen() const { return StaticGetDataLen(head.total); }

    ITP_Log_List_Head head;
    ITP_Log_File files[64 * 1000];
};

constexpr int ITPLogDataMaxLen = 8000;
struct ITP_Log_Context
{
    static uint16_t StaticGetDataLen(int len)
    {
        return sizeof(ITP_Log_Context) - ITPLogDataMaxLen + len;
    }
    bool checkLen(uint16_t len) const { return GetDataLen() == len; }
    uint16_t GetDataLen() const { return StaticGetDataLen(currentLen); }
    uint16_t index;
    uint16_t currentLen;
    uint32_t totalLen;
    uint8_t finish;
    uint8_t rfu[3];
    char data[ITPLogDataMaxLen];
};

struct ITP_Log_Level
{
    uint32_t level;
    qint8 rfu[16];
};

struct ITP_Log_Level_Ans
{
    uint32_t ret;
    qint8 rfu[16];
};

// --- Board Info and Broadcast Structures ---
typedef struct
{
    uint8_t bMacAddr[6];
    uint8_t chassisType;
    uint8_t chassisIndex;
    uint8_t chassisSlot;
    uint8_t subBoardIndex;
    uint8_t boardType;
    uint8_t memorySize;
    char systemName[16];
    char rfu2[16];
    char loginPassword[16];
    uint32_t localIP;
    uint32_t localIPMask;
    uint32_t gatewayIP;
    uint16_t listenPort;
    uint16_t ipValidFlag; // Should be 0x55AA
    char firmwareName[8];
    char firmwareVersion[8];
    char revisionDate[8];
} ITP_BroadcastXmsBoardInfo;

typedef struct
{
    quint8 _ec_version;
    quint8 _gy_boardType;
    quint8 rfu[2];
} rfu2_ext;

typedef struct
{
    uint8_t bMacAddr[6];
    uint8_t chassisType;
    uint8_t chassisIndex;
    uint8_t chassisSlot;
    uint8_t subBoardIndex;
    uint8_t boardType;
    uint8_t rfu1;
    char BIOSfirmwareVersion[8];
    char BIOSrevisionDate[8];
    uint16_t cpuLoad;
    uint16_t rfu3[1];
    uint32_t swi_5ms_miss_count;
    uint32_t sig_swi_5ms_miss_count;
    uint32_t swi_30ms_miss_count;
    char m_s8PlatformType;
    char m_s8UserCode[7];
    qint32 m_s32SysId;
    union
    {
        uint8_t _pcm_sub_type_list[4];
        rfu2_ext ext;
    } ext_flag;
#define pcm_sub_type_list ext_flag._pcm_sub_type_list
#define ec_version ext_flag.ext._ec_version
#define gy_boardType ext_flag.ext._gy_boardType
    uint32_t localIP;
    uint32_t localIPMask;
    uint32_t gatewayIP;
    uint16_t listenPort;
    uint16_t ipValidFlag;
    char firmwareName[8];
    char firmwareVersion[8];
    char revisionDate[8];
} ITP_UserReadXmsBoardInfo;

#define TOTAL_DIGITAL_PORTS 4
#define SUB_DSP_NUM_ONE_DSP (3)

typedef struct
{
    uint8_t u8IPMode;
    uint8_t u8reserved[3];
    uint32_t slaveIP[SUB_DSP_NUM_ONE_DSP];
    uint32_t slaveIPpre[SUB_DSP_NUM_ONE_DSP];
    uint32_t u32reserved[2];
    uint8_t MacAddr[SUB_DSP_NUM_ONE_DSP][8];
} Slave_IP_Config;

/*--FIXED: Moved ITP_Digital_port_Property definition here from Func_type=4 section to resolve compilation order--*/
typedef struct
{
    uint8_t m_u8Port_Type;
    uint8_t m_u8SS7_Link1_ts;
    uint8_t m_u8SS7_Link2_ts;
    uint8_t m_u8Voice_Ch_Enable;
    uint8_t m_u8Tx_Tristate;
    uint8_t m_u8Tx_Build_Out;
    uint8_t m_u8Tx_G703_Clock;
    uint8_t m_u8Rx_G703_Clock;
    uint8_t m_u8Tx_Termination;
    uint8_t m_u8Rx_Termination;
    uint8_t m_u8Rx_Sensitivity;
    uint8_t m_u8Rx_Monitor;
    uint8_t m_u8JA_Enable;
    uint8_t m_u8JA_Depth;
    uint8_t m_u8JA_Side;
    uint8_t m_u8E1_TAF;
    uint8_t m_u8E1_TNAF;
    uint8_t m_u8Auto_Alarm;
    uint8_t m_u8CRC_Mode;
    uint8_t m_u8CAS_MF_Byte;
    uint8_t m_u8Line_Code;
    uint8_t m_u8Loopback_Mode;
    uint8_t m_u8Dpj_Version;
    uint8_t m_u8E1Alarm_Enable;
} ITP_Digital_port_Property;

typedef struct
{
    ITP_BroadcastXmsBoardInfo boardInfo;
    ITP_Digital_port_Property ports[TOTAL_DIGITAL_PORTS];
    Slave_IP_Config gConfigSlaveIP;
} TAllDspInfoForConfig;

typedef struct
{
    char packetName[16];
    unsigned packetLength;
    TAllDspInfoForConfig config;
} ITP_SearchDspPacket;

typedef ITP_SearchDspPacket ITP_SetDspPacket;

// --- Resource and Capacity Structures ---
#define ITP_MAX_RESOURCE_NUMBER 128

#define ITP_MAX_RES_LIST_LENGTH 16
struct ITP_Res_Cap_List
{
    static constexpr uint16_t staticGetDataLen(uint16_t resListLength)
    {
        Q_ASSERT(resListLength <= ITP_MAX_RES_LIST_LENGTH);
        return sizeof(ITP_Res_Cap_List) - (ITP_MAX_RES_LIST_LENGTH - resListLength) * sizeof(ITP_Res_Cap);
    }
    bool checkLen(uint16_t dataLen) const
    {
        return getDataLen() == dataLen;
    }
    uint16_t getDataLen() const
    {
        return staticGetDataLen(m_u16Res_List_Length);
    }
    uint16_t m_u16Res_List_Length; /*说明后面跟了多少项资源数量定义*/
    uint16_t m_u16Reserved;        /*保留*/
    ITP_Res_Cap m_ITP_resource_capacity[ITP_MAX_RES_LIST_LENGTH];
};

#define ITP_MAX_RESOURCE_NUMBER_EIC 32
#define ITP_MAX_RESCH_NUM 256
typedef struct
{
    uint16_t m_u16ResNum;
    char m_rfu1[2];
    uint8_t m_u8ResType[ITP_MAX_RESCH_NUM];
    char m_rfu2[32];
} ITP_ResCap_Ch;

typedef struct
{
    ITP_ResCap_Ch m_ResCapCh[ITP_MAX_RESOURCE_NUMBER_EIC];
    char m_rfu[32];
} ITP_ResCap;

typedef struct
{
    uint8_t m_u8Module_Name[8];
    uint32_t m_u32Ver_Number;
} ITP_Res_Cap_Head;

struct GYDef_ITP_Res
{
    bool checkLen(uint16_t dataLen) const { return getDataLen() == dataLen; }
    uint16_t getDataLen() const { return sizeof(head) + list.getDataLen(); }
    ITP_Res_Cap_Head head;
    ITP_Res_Cap_List list;
};

// --- GTG/GTD Structures ---
typedef struct
{
    uint16_t m_u16Freq_Mask;
    uint16_t m_u16Amp_Threshold;
    uint16_t m_u16Envelope_Mode;
    uint16_t m_u16Repeat_Count;
    uint16_t m_u16Min_On_Time1;
    uint16_t m_u16Max_On_Time1;
    uint16_t m_u16Min_Off_Time1;
    uint16_t m_u16Max_Off_Time1;
    uint16_t m_u16Min_On_Time2;
    uint16_t m_u16Max_On_Time2;
    uint16_t m_u16Min_Off_Time2;
    uint16_t m_u16Max_Off_Time2;
} ITP_GTD_Prototype;

typedef struct
{
    uint8_t m_u8Freq1_Index;
    uint8_t m_u8Freq2_Index;
    uint16_t m_u16Freq1_Amp;
    uint16_t m_u16Freq2_Amp;
    uint16_t m_u16Envelope_Mode;
    uint16_t m_u16Repeat_Count;
    uint16_t m_u16On_Time1;
    uint16_t m_u16Off_Time1;
    uint16_t m_u16On_Time2;
    uint16_t m_u16Off_Time2;
    uint16_t m_u16Reserved;
} ITP_GTG_Prototype;

// --- Other Global Parameter Structures ---
typedef struct
{
    uint16_t rxFilterWindow;
    uint16_t rxPeriodTheshold;
    uint16_t rx55Count;
    uint16_t rxAllOneSamples;
    uint16_t rxDataTimeOut;
    uint16_t txBit1Freq;
    uint16_t txBit0Freq;
    uint16_t tx55Count;
    uint16_t txAllOneSamples;
    uint16_t reserved[3];
} ITP_FskParamsStructure;

typedef struct
{
    uint8_t MinHookFlashTime;
    uint8_t MaxHookFlashTime;
} HOOK_FLASH_PARAMS_STRUCTURE;

typedef struct
{
    uint8_t HookFlashTime;
    uint8_t reserved[3];
} TRUNK_HOOK_FLASH_PARAMS_STRUCTURE;

typedef struct
{
    uint32_t reserved[4];
} ITP_Reset_DSP_state;

typedef struct
{
    uint32_t m_u32SilenceLevel;
    uint32_t m_u32SilenceTimer;
    uint32_t m_u32Reserved;
} ITP_V_VSD_SET;

typedef struct
{
    uint16_t m_u16VadMinLevel;
    uint16_t m_u16VadInterval;
} VAD_Min_Level_Config;

typedef struct
{
    uint8_t m_u8IsCNGEnable;
    uint16_t m_u16CNGGainLevel;
    uint8_t m_u8Ref[29];
} Board_Comm_Param_Config;

//-----------------------------------------------------------------------------
// Func_type=2: Speech Functions (DSP_MAIN_FUNCTION_SPEECH)
//-----------------------------------------------------------------------------
// --- Command Sub-functions ---
#define ITP_SUBFUNC_SET_INPUT_CH_CONFIG 0x01
#define ITP_SUBFUNC_SET_OUTPUT_CH_CONFIG 0x02
#define ITP_SUBFUNC_SET_PLAY_CH_CONFIG 0x03
#define ITP_SUBFUNC_SET_RECORD_CH_CONFIG 0x04
#define ITP_SUBFUNC_SET_GTD_CH_CONFIG 0x05
#define ITP_SUBFUNC_SET_CONF_GRP_CONFIG 0x06
#define ITP_SUBFUNC_GET_INPUT_CH_CONFIG 0x07
#define ITP_SUBFUNC_GET_OUTPUT_CH_CONFIG 0x08
#define ITP_SUBFUNC_GET_PLAY_CH_CONFIG 0x09
#define ITP_SUBFUNC_GET_RECORD_CH_CONFIG 0x0A
#define ITP_SUBFUNC_GET_GTD_CH_CONFIG 0x0B
#define ITP_SUBFUNC_GET_CONF_GRP_CONFIG 0x0C
#define ITP_SUBFUNC_SEND_STREAM_DATA 0x10
#define ITP_SUBFUNC_SET_GTG_CH_CONFIG 0x12
#define ITP_SUBFUNC_SET_RTPOUT_CH_CONFIG 0x14
#define ITP_SUBFUNC_SET_RTPIN_CH_CONFIG 0x15
#define ITP_SUBFUNC_SET_FULLVOC_CH_CONFIG 0x16
#define ITP_SUBFUNC_SET_324_CH_CONFIG 0x17
#define ITP_SUBFUNC_SET_324_MIXER 0x19
#define ITP_SUBFUNC_SET_VOC_FSK_MODE 0x21
#define ITP_SUBFUNC_GET_INPUT_CH_PROPERTY 0x22
#define ITP_SUBFUNC_SET_FLUSH_BUF_CONFIG 0x30

// --- Event Sub-functions ---
#define ITP_SUBFUNC_INPUT_CH_CONFIG 0x07
#define ITP_SUBFUNC_OUTPUT_CH_CONFIG 0x08
#define ITP_SUBFUNC_PLAY_CH_CONFIG 0x09
#define ITP_SUBFUNC_RECORD_CH_CONFIG 0x0A
#define ITP_SUBFUNC_GTD_CH_CONFIG 0x0B
#define ITP_SUBFUNC_CONF_GRP_CONFIG 0x0C
#define ITP_SUBFUNC_PLAY_CH_EVENT 0x0D
#define ITP_SUBFUNC_RECORD_CH_EVENT 0x0E
#define ITP_SUBFUNC_GTD_CH_EVENT 0x0F
#define ITP_SUBFUNC_RCV_STREAM_DATA 0x10
#define ITP_SUBFUNC_SEND_STREAM_DATA_REQ 0x11
#define ITP_SUBFUNC_GTG_CH_EVENT 0x13
#define ITP_SUBFUNC_RECORD_CSP_CH_EVENT 0x14
#define ITP_SUBFUNC_PLAY_3GPP_CH_EVENT 0x15
#define ITP_SUBFUNC_RECORD_3GPP_CH_EVENT 0x16
#define ITP_SUBFUNC_GET_3GPP_CFG_EVENT 0x17
#define ITP_SUBFUNC_INIT_3GPP_INDEX_EVENT 0x18
#define ITP_SUBFUNC_BUILD_3GPP_INDEX_EVENT 0x19
#define ITP_SUBFUNC_FSK_CH_EVENT 0x20
#define ITP_SUBFUNC_BUILD_INDEX_EVENT 0x21
#define ITP_SUBFUNC_RCV_FORWARD_EVENT 0x22
#define ITP_SUBFUNC_RCV_BACKWARD_EVENT 0x23
#define ITP_SUBFUNC_RCV_REC_BACKWARD_EVENT 0x24

// --- Speech Constants ---
#define ITP_VOC_CH_DISABLE 0x00
#define ITP_VOC_CH_ENABLE 0x01
#define ITP_VOC_CH_DELAY_STOP 0x02
#define ITP_VOC_CH_PLAY_PAUSE 0x05
#define ITP_VOC_CH_PLAY_RESUME 0x06
#define ITP_VOC_CH_PLAY_FORWARD 0x09
#define ITP_VOC_CH_PLAY_BACKWARD 0x0A
#define ITP_VOC_CH_REC_PAUSE 0x05
#define ITP_VOC_CH_REC_RESUME 0x06
#define ITP_VOC_CH_REC_BACKWARD 0x0A
#define ITP_GTD_DISABLE 0x00
#define ITP_GTD_ON_INPUT 0x01
#define ITP_GTD_ON_OUTPUT 0x02
#define ITP_MIXER_FROM_NULL 0x00
#define ITP_MIXER_FROM_INPUT 0x01
#define ITP_MIXER_FROM_PLAY 0x02
#define ITP_MIXER_FROM_CONF 0x03
#define ITP_MIXER_FROM_RTP 0x04
#define ITP_MIXER_FROM_H324 0x05
#define ITP_MIXER_FROM_RTPX 0x06
#define ITP_MIXER_FROM_OUTPUT 0x06

#define ITP_GTG_GEN_DTMF 0x00
#define ITP_GTG_GEN_MR2F 0x01
#define ITP_GTD_CODETYPE_DTMF 0x01
#define ITP_GTD_CODETYPE_MR2F 0x02
#define ITP_CONF_OPERATION_ADD 0x1
#define ITP_CONF_OPERATION_REMOVE 0x2
#define ITP_CONF_OPERATION_CLEAR 0x3

// --- Speech Structures ---
typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8AGC_Enable;
    uint8_t m_u8AGC_Mode;
    uint8_t m_u8EC_Enable;
    uint8_t m_u8EC_Ref_Type;
    uint8_t m_u8Conf_Enable;
    uint8_t m_u8Conf_VAD;
    uint8_t m_u8Conf_TAD;
    uint16_t m_u16EC_Ref_ID;
    uint16_t m_u16Conf_Group;
    uint16_t m_u16Fix_Gain;
    uint8_t m_u8NR_Enable;
    uint8_t m_u8Reserved;
} ITP_Input_ch_Property;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8AGC_Enable;
    uint8_t m_u8AGC_Mode;
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint8_t m_u8Reserved;
    uint16_t m_u16SRC1_ID;
    uint16_t m_u16SRC2_ID;
    uint16_t m_u16Fix_Gain;
} ITP_Output_ch_Property;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8SRC_Mode;
    uint8_t m_u8Conf_Enable;
    uint8_t m_u8Decode_Type;
    uint16_t m_u16Conf_Group;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8Stop_Mode;
    uint8_t m_u8Stop_Code;
    uint8_t m_u8Tag_Number_last;
    uint16_t m_u16Stop_Ref_ch;
} ITP_Play_ch_Property;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8SRC_Mode;
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint8_t m_u8Encode_Type;
    uint8_t m_u8recordType;
    uint8_t m_u8Reserved1[2];
    uint16_t m_u16SRC1_ID;
    uint16_t m_u16SRC2_ID;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8Stop_Mode;
    uint8_t m_u8Stop_Code;
    uint8_t m_u8Tag_Number_last;
    uint16_t m_u16Stop_Ref_ch;
    uint16_t m_u16Stream_Packet_Len;
} ITP_Record_ch_Property;

#define ITP_MAX_GTD_LENGTH 1024
struct ITP_GTD_ch_Event
{
    bool checkLen(uint16_t dataLen) const { return getDataLen() == dataLen; }
    uint16_t getDataLen() const { return sizeof(*this) - sizeof(m_u8GTD_ID_Buf) + m_u16GTD_Length; }
    uint16_t m_u16GTD_Length;
    uint8_t m_u8CodeType;
    uint8_t m_u8Reserved;
    uint8_t m_u8GTD_ID_Buf[ITP_MAX_GTD_LENGTH];
};

#define ITP_MAX_STREAM_DATA_LENGTH 4000
typedef struct
{
    uint16_t m_u16data_length;
    uint8_t m_u8data_type;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8stream_data[ITP_MAX_STREAM_DATA_LENGTH];
} ITP_Send_Stream_Data;

typedef ITP_Send_Stream_Data ITP_Rev_Stream_Data;

typedef struct
{
    uint16_t m_u16remain_space;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8data_type;
} ITP_Send_Stream_Data_Req;

//-----------------------------------------------------------------------------
// Func_type=4: Digital Trunk Functions (DSP_MAIN_FUNCTION_DIGITAL)
//-----------------------------------------------------------------------------
// --- Command Sub-functions ---
#define ITP_SUBFUNC_SET_DIGITAL_PORT_CONFIG 0x01
#define ITP_SUBFUNC_GET_DIGITAL_PORT_CONFIG 0x02
#define ITP_SUBFUNC_GET_DIGITAL_PORT_STATE 0x03

// --- Event Sub-functions ---
#define ITP_SUBFUNC_DIGITAL_PORT_CONFIG 0x02
#define ITP_SUBFUNC_DIGITAL_PORT_STATE_EVENT 0x03

// --- Digital Port Constants ---
#define ITP_Disable 0
#define ITP_Enable 1
#define ITP_E1_PCM31 2
#define ITP_E1_PCM30 3
#define ITP_E1_CAS 4
#define ITP_E1_PRI 5
#define ITP_E1_SS7_TUP_1_Link 7
#define ITP_E1_SS7_ISUP_1_Link 15
#define ITP_Output_Tristate 0
#define ITP_Output_Enable 1
#define ITP_Tx_75 1
#define ITP_Tx_120 3
#define ITP_Rx_75 1
#define ITP_Rx_120 3
#define ITP_E1_HDB3 0
#define ITP_E1_AMI 1
#define ITP_Normal_Working 0
#define ITP_Local_Loopback 3
#define ITP_Remote_Loopback 4

// --- Digital Port Structures ---
/*--FIXED: Definition for ITP_Digital_port_Property moved to Func_type=1 section to resolve dependency--*/

typedef struct
{
    uint8_t m_u8Lost_signal;
    uint8_t m_u8Fas_align;
    uint8_t m_u8Mf_align;
    uint8_t m_u8Crc4_align;
    uint8_t m_u8Remote_alarm;
    uint8_t m_u8Remote_MF_alarm;
    uint8_t m_u8Rx_level;
    uint8_t m_u8Tx_open;
    uint8_t m_u8Tx_overlimit;
    uint8_t m_u8Port_Current_State;
    uint8_t m_u8Port_Normal_Count;
    uint8_t m_u8Port_Error_Count;
    uint32_t m_u32total_seconds;
    uint32_t m_u32RLOS_seconds;
    uint32_t m_u32LRCL_seconds;
    uint32_t m_u32RUA1_seconds;
    uint32_t m_u32RRA_seconds;
    uint32_t m_u32RDMA_seconds;
    uint32_t m_u32JALT_seconds;
    uint32_t m_u32TOCD_seconds;
    uint32_t m_u32TCLE_seconds;
    uint32_t m_u32RSLIP_seconds;
    uint32_t m_u32TSLIP_seconds;
    uint32_t m_u32LCVCR_count;
    uint32_t m_u32PCVCR_count;
    uint32_t m_u32FOSCR_count;
    uint32_t m_u32EBCR_count;
    uint32_t m_u32HDLC1_Tx_packets;
    uint32_t m_u32HDLC1_Rx_goodpk;
    uint32_t m_u32HDLC1_Rx_badpk;
    uint32_t m_u32HDLC2_Tx_packets;
    uint32_t m_u32HDLC2_Rx_goodpk;
    uint32_t m_u32HDLC2_Rx_badpk;
    uint8_t m_u8E1Type;
    uint8_t m_u8Rfu[3];
} ITP_Digital_port_State;

//-----------------------------------------------------------------------------
// Func_type=5: Interface Channel Functions (DSP_MAIN_FUNCTION_INTERFACE)
//-----------------------------------------------------------------------------
// --- DSS1/Interface Sub-function IDs ---
#define DSP_SUB_FUNCTION_DSS1_CFG_CMD_TO_DSP 0x01      // Q931->DSP
#define DSP_SUB_FUNCTION_DSS1_GET_CFG_TO_DSP 0x02      // Q931->DSP
#define DSP_SUB_FUNCTION_DSS1_CFG_INFO_TO_Q931 0x02    // DSP->Q931
#define DSP_SUB_FUNCTION_DSS1_CONTROL_CMD_TO_DSP 0x03  // Q931->DSP
#define DSP_SUB_FUNCTION_DSS1_STATE_EVENT_TO_Q931 0x03 // DSP->Q931
#define DSP_SUB_FUNCTION_DSS1_SIGNAL_DATA_TO_DSP 0x04  // Q931->DSP
#define DSP_SUB_FUNCTION_DSS1_SIGNAL_DATA_TO_Q931 0x04 // DSP->Q931
#define DSP_SUB_FUNCTION_DSS1_DATA_REQ_TO_Q931 0x05    // DSP->Q931
#define DSP_SUB_FUNCTION_DSS1_DEBUG_DATA_TO_Q931 0x06  // DSP->Q931

// --- DSS1 Link Constants ---
#define ITP_DSS1_LINK_COMMAND_ESTABLISH 0x03
#define ITP_DSS1_LINK_COMMAND_RELEASE 0x05
#define ITP_DSS1_LINK_COMMAND_STOP 0x20
#define ITP_DSS1_LINK_STATE_ESTABLISH 0x03
#define ITP_DSS1_LINK_STATE_RELEASE 0x05
#define ITP_DSS1_LINK_STATE_NORMAL 0x0A
#define ITP_DSS1_LINK_STATE_ABNORMAL 0x0B
#define ITP_MAX_LINK_DATA_LENGTH 4000
#define ITP_MAX_LINK_DEBUG_DATA_LENGTH 64

// --- DSS1 Structures ---
typedef struct
{
    uint8_t m_u8NET;
    uint8_t m_u8AutoGetTEI;
    uint8_t m_u8Debug_Enable;
    uint8_t m_u8Reserved;
    uint16_t m_u16Debug_Interval;
    uint16_t m_u16ts_index;
} ITP_DSS1_link_Property;

struct ITP_DSS1_link_Send_Data
{
    bool checkLen(uint16_t dataLen) const { return getDataLen() == dataLen; }
    uint16_t getDataLen() const { return sizeof(*this) - sizeof(data) + m_u16data_length; }
    uint16_t m_u16data_length;
    uint16_t m_u16data_type;
    uint8_t data[ITP_MAX_LINK_DATA_LENGTH];
};
typedef ITP_DSS1_link_Send_Data ITP_DSS1_link_Rev_Data;

//... Other interface-related structures and constants ...
// From _donjin_dsp_def.h
#define ITP_SUBFUNC_SET_INTERFACE_CH_CONFIG 0x01
#define ITP_SUBFUNC_GET_INTERFACE_CH_CONFIG 0x02
#define ITP_SUBFUNC_GET_RX_STATE 0x03
#define ITP_SUBFUNC_SET_INTERFACE_CH_VOLTAGE 0x07
#define ITP_SUBFUNC_GET_INTERFACE_CH_SN 0x08
#define ITP_SUBFUNC_READ_INTERFACE_CH_VOLTAGE 0x09
#define ITP_SUBFUNC_INTERFACE_CH_CONFIG 0x02
#define ITP_SUBFUNC_INTERFACE_CH_EVENT 0x03
#define ITP_SUBFUNG_DIGITAL_REC_CH_CONFIG 0x04
#define ITP_SUBFUNC_DIGITAL_REC_CH_EVENT 0x05
#define ITP_SUBFUNC_DIGITAL_REC_DCH_DATA 0x06
#define ITP_SUBFUNC_INTERFACE_CH_VOLTAGE 0x07
#define ITP_SUBFUNC_INTERFACE_CH_SN 0x08

//-----------------------------------------------------------------------------
// Func_type=B: Firmware Operations (DSP_MAIN_FUNCTION_FIRMWARE)
//-----------------------------------------------------------------------------
#define DSP_SUB_FUNCTION_FIRMWARE_READ_FLASH 0x01
#define DSP_SUB_FUNCTION_FIRMWARE_WRITE_FLASH 0x02
#define DSP_SUB_FUNCTION_FIRMWARE_ERASE_FLASH 0x03
#define DSP_SUB_FUNCTION_FIRMWARE_FINISH_FLASH 0x04
#define DSP_SUB_FUNCTION_FIRMWARE_READ_SDRAM 0x06
#define DSP_SUB_FUNCTION_REBOOT 0x07

#define ITP_UPDATE_ERASE_ID 0x87654321
#define ITP_WRITE_FLASH_OK 0x1
#define ITP_ERASE_FLASH_OK 0x1
#define ITP_UPDATE_FINISH_OK 0x1

#define ITP_MAX_DATA_BLOCK_LENGTH 4000

typedef struct
{
    uint32_t m_u32AddrOffset;
    uint32_t m_u32ByteNum;
} ITP_T_FLASH_READ;

typedef struct
{
    uint32_t m_u32data_len;
    uint8_t m_u8Data[ITP_MAX_DATA_BLOCK_LENGTH];
} ITP_T_FLASH_READBACK;

typedef struct
{
    uint32_t m_u32AddrOffset;
    uint32_t m_u32ByteNum;
    uint8_t m_u8Data[ITP_MAX_DATA_BLOCK_LENGTH];
} ITP_T_FLASH_WRITE;

#define ITP_T_FLASH_WRITE_LEN(data) (sizeof(*data) - sizeof(data->m_u8Data) + data->m_u32ByteNum)

typedef struct
{
    uint32_t m_u32write_status;
    uint32_t m_u32check_sum;
} ITP_T_FLASH_WRITE_EVENT;

//-----------------------------------------------------------------------------
// Func_type=0x0C: VoIP functions (DSP_MAIN_FUNCTION_VOIP)
//-----------------------------------------------------------------------------
#define ITP_DSP_SUBFUNC_SET_VOIP_CH_PARAMS 0x01
#define DSP_SUB_FUNCTION_VOIP_RTP_INIT 0x00
#define DSP_SUB_FUNCTION_VOIP_RTP_OPEN 0x01
#define DSP_SUB_FUNCTION_VOIP_RTP_CLOSE 0x02
#define DSP_SUB_FUNCTION_VOIP_SET_RTP_ADDR_A 0x03
#define DSP_SUB_FUNCTION_VOIP_SET_CODEC_TX_A 0x07
#define DSP_SUB_FUNCTION_VOIP_SET_CODEC_RX_A 0x08
#define DSP_SUB_FUNCTOIN_VOIP_SET_RTP_PARAM 0x10

/* 语音编码类型*/
#define VOIP_MEDIA_AUDIO_PCMU 0x00 /* g.711 u-law*/
#define VOIP_MEDIA_AUDIO_PCMA 0x01 /* g.711 a-law*/
#define VOIP_MEDIA_AUDIO_G723 0x02 /* g.723*/
#define VOIP_MEDIA_AUDIO_G729 0x03 /* g.729*/

/* RTP包负载类型*/
#define VOIP_MEDIA_AUDIO_PCMU_PT 0x00 /* g.711 u-law*/
#define VOIP_MEDIA_AUDIO_PCMA_PT 0x08 /* g.711 a-law*/
#define VOIP_MEDIA_AUDIO_G723_PT 0x04 /* g.723*/
#define VOIP_MEDIA_AUDIO_G729_PT 0x12 /* g.729*/

typedef struct
{
    uint16_t samples;
} VOIP_G711_PARAM;

typedef struct
{
    uint8_t frames;
    uint8_t bps;
    uint8_t hp;
    uint8_t pf;
    uint8_t vad;
} VOIP_G723_PARAM;

typedef struct
{
    uint8_t frames;
    uint8_t vad;
} VOIP_G729_PARAM;

typedef struct
{
    uint8_t enable_rtcp;
    uint8_t nat_traversal;
    uint32_t gateway_ip;
    uint16_t port_base;
    uint16_t close_inactive_sec;
    uint8_t ip_tos;
    uint8_t reserved[1];
    uint16_t jitter_mode;
    uint16_t jitter_time;
    uint16_t tune_delay;
    uint8_t fraction_lost_lo;
    uint8_t fraction_lost_hi;
    VOIP_G711_PARAM g711;
    VOIP_G723_PARAM g723;
    VOIP_G729_PARAM g729;
} VOIP_RTP_CONFIG;

#define VOIP_RTP_PARAM_JITTER 1
#define VOIP_RTP_PARAM_G729 2
#define VOIP_RTP_PARAM_ENCRYPT 3
#define VOIP_RTP_PARAM_DECRYPT 4
#define VOIP_RTP_PARAM_AUDIO_SSRC 5
#define VOIP_RTP_PARAM_OUTPUT_VAD 6 // 2011.12.6 added
#define VOIP_RTP_PARAM_G711 7

typedef struct VOIP_OUTPUT_VAD_PARAM_TAG
{
    quint8 enable;
    quint8 rfu;
    quint16 active_tail_length; //*5ms
} VOIP_OUTPUT_VAD_PARAM;

typedef struct VOIP_JITTER_PARAM_TAG
{                        // 针对dsp
    quint16 jitter_mode; // jitter buffer mode (0 - static, 1 - adaptive)
    quint16 jitter_time; // jitter buffer size in milli-seconds (static mode)
} VOIP_JITTER_PARAM;

typedef struct VOIP_ENCRYPT_PARAM_TAG // 针对通道
{
    quint8 enable;
    quint8 rfu[3];
    quint8 key[8];
} VOIP_ENCRYPT_PARAM;

typedef struct AUDIO_SSRC_SET_TAG
{
    quint32 ssrc;
    quint32 reserved;
} AUDIO_SSRC_SET;

typedef struct RTP_PARAM_TAG
{
    quint8 type;
    quint8 rfu[3];
    union PARAM_TAG
    {
        VOIP_JITTER_PARAM jitter;
        VOIP_G729_PARAM g729;
        VOIP_ENCRYPT_PARAM encrypt;
        AUDIO_SSRC_SET audio_ssrc;       // add by zcq at 20111109
        VOIP_OUTPUT_VAD_PARAM outputVad; // add by lyp at 2011.12.6
        VOIP_G711_PARAM g711;
    } param;
} RTP_PARAM;

//=============================================================================
// X. Resource Type Definitions and Miscellaneous
//=============================================================================

//-----------------------------------------------------------------------------
// X.1 Resource Type Definitions
//-----------------------------------------------------------------------------
#define ITP_UNKNOWN_GY_DEF 0x00
#define ITP_DSP_GY_DEF 0x01
#define ITP_VOC_CH_TYPE 0x02
#define ITP_FAX_CH_TYPE 0x03
#define ITP_DIGITAL_PORT_TYPE 0x04
#define ITP_INTERFACE_CH_TYPE 0x05
#define ITP_DSS1_LINK_TYPE 0x06
#define ITP_SS7_LINK_TYPE 0x07
#define ITP_CTBUS_CLK_TYPE 0x08
#define ITP_CTBUS_TS_TYPE 0x09
#define ITP_RTP_CH_TYPE 0x0a
#define ITP_MEDIAEX_CH_TYPE 0x0b
#define ITP_CONF_GROUP_TYPE 0x0c
#define ITP_H324M_CH_TYPE 0x0d
#define ITP_HIZ_SS764K_TYPE 0x0e
#define ITP_HIZ_SS72M_TYPE 0x0f
#define ITP_EXCHANGE_CH_TYPE 0x10
#define ITP_RTPX_CH_TYPE 0x11
#define ITP_VOIP_CH_TYPE 0x12
#define ITP_VIDEOCONF_GROUP_TYPE 0x1f
#define ITP_CH_TYPE_GY_DEF_COM 0x80

//-----------------------------------------------------------------------------
// X.2 Alarms
//-----------------------------------------------------------------------------
#define Device_AlarmType 0x00
#define Connect_AlarmType 0x01
#define Properties_AlarmType 0x02
#define Link_AlarmType 0x03

#define NORMAL_AlarmLevel 0x00
#define Serious_AlarmLevel 0x01

struct ITP_ALARM_Info
{
    uint8_t m_u8AlarmType;
    uint8_t m_u8AlarmLevel;
    char AlarmInfo[64];
};

//=============================================================================
// XX. Definitions Imported from DSP_PACKET_DEF.H
//=============================================================================

//-----------------------------------------------------------------------------
// XX.1 Global Definitions
//-----------------------------------------------------------------------------

// --- Main Function IDs (alternative names) ---
#define ITP_DSP_MAINFUNC_BOARD_LEVEL 0x01
#define ITP_DSP_MAINFUNC_VOC_CHANNEL 0x02
#define ITP_DSP_MAINFUNC_FAX_CHANNEL 0x03
#define ITP_DSP_MAINFUNC_E1_PORT 0x04
#define ITP_DSP_MAINFUNC_INTERFACE_CHANNEL 0x05
#define ITP_DSP_MAINFUNC_PRI_LINK 0x06
#define ITP_DSP_MAINFUNC_SS7_LINK 0x07
#define ITP_DSP_MAINFUNC_CTBUS_CLK 0x08
#define ITP_DSP_MAINFUNC_CTBUS_TS 0x09
#define ITP_DSP_MAINFUNC_TS_CONNECTION 0x0A
#define ITP_DSP_MAINFUNC_UPDATE_FIRMWARE 0x0B
#define ITP_DSP_MAINFUNC_VOIP_CHANNEL 0x0C
#define ITP_DSP_MAINFUNC_VIDEOAUDIO_CHANNEL 0x0D
#define ITP_DSP_MAINFUNC_CONF_GROUP 0x0E

// --- IO Data Event Types ---
#define ITP_SEND_IODATA_EVT_GTG_FINISH 0x00
#define ITP_SEND_IODATA_EVT_GTG_STOP 0x01
#define ITP_SEND_IODATA_EVT_GTG_SIG 0x02
#define ITP_SEND_IODATA_EVT_FSK 0x03
#define ITP_SEND_IODATA_EVT_MSK 0x04

// --- EC Versions ---
#define EC_VERSION_1 0x00
#define EC_VERSION_2 0x01

// --- Misc Structures ---
typedef struct
{
    uint8_t bMacAddr[6];
    uint8_t chassisType;
    uint8_t chassisIndex;
    uint8_t chassisSlot;
    uint8_t subBoardIndex;
    uint8_t boardType;
    uint8_t memorySize;
    int8_t systemName[16];
    int8_t rfu2[16];
    int8_t loginPassword[16];
    uint32_t localIP;
    uint32_t localIPMask;
    uint32_t gatewayIP;
    uint16_t listenPort;
    uint16_t ipValidFlag;
    int8_t firmwareName[8];
    int8_t firmwareVersion[8];
    int8_t revisionDate[8];
} XmsBoardInfoForChangeDspIP;

typedef struct
{
    uint16_t m_u16Freq_Index;
    uint16_t m_u16Freq_Coef;
} ITP_GTG_Frequency;

typedef struct
{
    uint16_t m_u16Freq_Index;
    uint16_t m_u16Freq_Coef;
} ITP_GTD_Frequency;

typedef struct
{
    uint8_t m_u8GTG_id;
    uint8_t m_u8Reserved1;
    uint16_t m_u16Reserved2;
    ITP_GTG_Prototype m_ITP_gtg_prototype;
} ITP_GTG_Template;

typedef struct
{
    uint8_t m_u8GTD_id;
    uint8_t m_u8Reserved1;
    uint16_t m_u16Reserved2;
    ITP_GTD_Prototype m_ITP_gtd_prototype;
} ITP_GTD_Template;

typedef struct
{
    uint16_t m_u16Freq_Coef[16];
    uint16_t m_u16Reserved;
    ITP_GTG_Prototype m_ITP_gtg_prototype[16];
} ITP_GTG_Global_Parm;

typedef struct
{
    uint16_t m_u16Freq_Coef[16];
    uint16_t m_u16Reserved;
    ITP_GTD_Prototype m_ITP_gtd_prototype[16];
} ITP_GTD_Global_Parm;

typedef struct
{
    uint8_t m_u8Res_ID;
    uint8_t m_u8reserved[3];
} ITP_Get_Res_Ch_Comm;

typedef struct
{
    uint8_t m_u8Res_ID;
    uint8_t m_u8reserved;
    uint16_t m_u16total_number;
    uint8_t m_u8ch_type_list[ITP_MAX_RESOURCE_NUMBER];
} ITP_Res_Ch_Type_List;

//-----------------------------------------------------------------------------
// XX.2 Speech Function Definitions (Func_type=2)
//-----------------------------------------------------------------------------

// --- Voice channel subtype ---
#define ITP_VOC_SUBTYPE_NORMAL 0x00
#define ITP_VOC_SUBTYPE_H324M 0x01

// --- Command Sub-functions ---
#define ITP_SUBFUNC_GET_INPUT_CH_PROPERTY_3Dot0 0x24

// --- Event-related Structures ---
typedef struct
{
    uint16_t m_u16BufDataLen;
    uint8_t m_u8TagNumber;
    uint8_t m_u8DataType;
    uint8_t m_u8For_Back_Flag;
    uint8_t m_u8Reserved[3];
} ITP_PlayBuf_Data_Len;

typedef struct
{
    uint16_t m_u16BufDataLen;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8Data_Type;
    uint8_t m_u8For_Back_Flag;
    uint8_t m_u8Reserved[3];
} ITP_RecordBuf_Data_Len;

// --- Speech Constants ---
#define ITP_VOC_CH_CSP_ENABLE 0x03
#define ITP_VOC_CH_CSP_ENABLE_NO_STOP_PLAY 0x04
#define ITP_VOC_CH_TTS_ENABLE 0x04 /* Note: Duplicate value */
#define ITP_VOC_CH_NULL 0x08
#define ITP_VOC_CH_CSP_ENABLE_VOIP 0x13
#define ITP_VOC_CH_CSP_ENABLE_NO_STOP_PLAY_VOIP 0x14

#define ITP_AGC_MODE_ALS 0x00
#define ITP_AGC_MODE_AGC 0x01
#define ITP_EC_REF_FROM_INPUT 0x00
#define ITP_EC_REF_FROM_OUTPUT 0x01

#define ITP_OUTPUT_SILENCE 0x00
#define ITP_OUTPUT_FROM_INPUT 0x01
#define ITP_OUTPUT_FROM_PLAY 0x02
#define ITP_OUTPUT_FROM_MIXER 0x03
#define ITP_OUTPUT_UNCHANGE 0x04

#define ITP_PLAY_SRC_8K 0x00
#define ITP_PLAY_SRC_6K 0x01
#define ITP_PLAY_SRC_GTG 0x02
#define ITP_PLAY_SRC_FSK 0x03
#define ITP_PLAY_SRC_RTP 0x04
#define ITP_PLAY_SRC_FAX 0x05
#define ITP_PLAY_SRC_3GVIDEO 0x06
#define ITP_PLAY_SRC_11K 0x07
#define ITP_PLAY_SRC_ECM_FAX 0x08
#define ITP_PLAY_SRC_MSK 0x09

#define ITP_RECORD_AS_8K 0x00
#define ITP_RECORD_AS_6K 0x01
#define ITP_RECORD_AS_FSK 0x03
#define ITP_RECORD_AS_RTP 0x04
#define ITP_RECORD_AS_FAX 0x05
#define ITP_RECORD_AS_3GVIDEO 0x06
#define ITP_RECORD_AS_11K 0x07
#define ITP_RECORD_AS_ECM_FAX 0x08

#define ITP_PLAY_ENCODE_ALAW 0x00
#define ITP_PLAY_ENCODE_ULAW 0x01
#define ITP_PLAY_ENCODE_VOX 0x02
#define ITP_PLAY_8BIT_LINEAR 0x03
#define ITP_ENCODE_16BIT_LINEAR 0x04
#define ITP_ENCODE_G729 0x05
#define ITP_ENCODE_G723 0x06

#define ITP_STOP_MODE_NORMAL 0x00
#define ITP_STOP_AT_ONE_GTD 0x01
#define ITP_STOP_AT_ANY_GTD 0x02

#define ITP_RECORD_ENCODE_ALAW 0x00
#define ITP_RECORD_ENCODE_ULAW 0x01
#define ITP_RECORD_ENCODE_VOX 0x02
#define ITP_RECORD_8BIT_LINEAR 0x03

#define ITP_AUDIO_ENCODE_AMR 0x05
#define ITP_AUDIO_ENCODE_G7231 0x06
#define ITP_PCM_CODE_STREAM 0x07
#define ITP_VIDEO_ENCODE_TYPE_H261 0x00
#define ITP_VIDEO_ENCODE_TYPE_H263P 0x01
#define ITP_VIDEO_ENCODE_TYPE_H263 0x02
#define ITP_VIDEO_ENCODE_TYPE_MP4V 0x03
#define ITP_VIDEO_ENCODE_TYPE_H264 0x04

#define ITP_GTG_GEN_MR2B 0x02
#define ITP_GTG_GEN_GTG 0x03
#define ITP_GTG_GEN_FSK 0x04
#define ITP_GTG_GEN_MSK 0x05

#define ITP_GTD_CODETYPE_MR2B 0x03
#define ITP_GTD_CODETYPE_GTD 0x04
#define ITP_GTD_CODETYPE_FSK 0x05
#define ITP_GTD_CODETYPE_DPD 0x06
#define ITP_GTD_CODETYPE_PVD 0x07
#define ITP_GTD_CODETYPE_MSK 0x08
#define ITP_GTD_CODETYPE_VSD 0x09
#define ITP_GTD_CODETYPE_MSK_RECV_END 0x0a

#define ITP_CONF_SRCTYPE_SILENCE 0x0
#define ITP_CONF_SRCTYPE_VOCINPUT 0x1
#define ITP_CONF_SRCTYPE_VOCPLAY 0x2
#define ITP_CONF_SRCTYPE_RTPINPUT 0x3
#define ITP_CONF_SRCTYPE_CONFOUT 0x4
#define ITP_CONF_SRCTYPE_RTPXINPUT 0x5
#define ITP_CONF_INPUT_SILENCE 0x0
#define ITP_CONF_INPUT_NORMAL 0x1
#define ITP_CONF_INPUT_PLAY 0x2
#define ITP_CONF_OUTPUT_SILENCE 0x0
#define ITP_CONF_OUTPUT_NORMAL 0x1
#define ITP_CONF_OUTPUT_SUM 0x2
#define ITP_CONF_OPERATION_GET_ACTIVE_MEMBER 0x4

// --- Speech Structures ---
typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8AudioSRC1_Ctrl;
    uint8_t m_u8AudioSRC2_Ctrl;
    uint8_t m_u8VideoSRC1_Ctrl;
    uint16_t m_u16AudioSRC1_ID;
    uint16_t m_u16AudioSRC2_ID;
    uint16_t m_u16VideoSRC1_ID;
    uint8_t m_u8AudioInDecodeFlag;
    uint8_t m_u8AudioOutEncodeFlag;
    uint8_t m_u8AudioInCodec;
    uint8_t m_u8AudioOutCodec;
    uint8_t m_u8VideoInCodec;
    uint8_t m_u8VideoOutCodec;
} ITP_H223_ch_Property;

struct ITP_Output_ch_Property_ForSave
{
    uint32_t valid;
    ITP_Output_ch_Property property;
};

typedef struct
{
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint16_t m_u16SRC_ChID1;
    uint16_t m_u16SRC_ChID2;
    uint8_t m_u8Video;
    int8_t m_s8Rfu;
} MixerControl_t;

#define ITP_PLAY_AUDIO 0x0
#define ITP_PLAY_VIDEO 0x1
#define ITP_PLAY_AUDIONVIDEO 0x4

#define ITP_PLAY2DSP_STOP 0x0
#define ITP_PLAY2DSP_START 0x1
#define ITP_PLAY2DSP_DELAY_STOP 0x2
#define ITP_PLAY2DSP_PAUSE 0x5
#define ITP_PLAY2DSP_RESUME 0x6
#define ITP_PLAY2DSP_RESERVE 0x7
#define ITP_PLAY2DSP_NULL 0x8
#define ITP_PLAY2DSP_FORWARD 0x9
#define ITP_PLAY2DSP_BACKWARD 0xA

#define ITP_RECORD2DSP_STOP 0x0
#define ITP_RECORD2DSP_START 0x1
#define ITP_RECORD2DSP_DELAY_STOP 0x2
#define ITP_RECORD2DSP_PAUSE 0x5
#define ITP_RECORD2DSP_RESUME 0x6
#define ITP_RECORD2DSP_NULL 0x8
#define ITP_RECORD2DSP_FORWARD 0x9
#define ITP_RECORD2DSP_BACKWARD 0xA

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8SRC_Mode;
    uint8_t m_u8Conf_Enable;
    uint8_t m_u8Decode_Type;
    uint16_t m_u16Conf_Group;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8Stop_Mode;
    uint8_t m_u8Stop_Code;
    uint8_t m_u8Tag_Number_last;
    uint16_t m_u16Stop_Ref_ch;
    uint8_t m_u8MediaType;
    uint8_t m_u8NoDecode;
    uint8_t m_u8Rfu[2];
    uint8_t m_u8Video_Enable;
    uint8_t m_u8VideoSRC_Mode;
    uint8_t m_u8VideoConf_Enable;
    uint8_t m_u8VideoDecode_Type;
    uint16_t m_u16VideoConf_Group;
    uint8_t m_u8VideoTag_Number;
    uint8_t m_u8VideoStop_Mode;
    uint8_t m_u8VideoStop_Code;
    uint8_t m_u8VideoFrameRate;
    uint16_t m_u16VideoStop_Ref_ch;
    uint16_t m_u16VideoHeight;
    uint16_t m_u16VideoWidth;
} ITP_Play_ch_Property_3dot0;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8Reserved0;
    uint8_t m_u8Conf_VAD;
    uint8_t m_u8Decode_Type;
    uint16_t m_u16Reserved;
    uint8_t m_u8Delay_Countdown;
    uint8_t m_u8Reserved1;
} ITP_RTPIN_ch_Property;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8Video;
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint8_t m_u8Encode_Type;
    uint8_t m_u8Delay_Countdown;
    uint8_t m_u8VideoSRC_Ctrl;
    uint8_t m_u8VideoSRC_ID;
    uint16_t m_u16SRC1_ID;
    uint16_t m_u16SRC2_ID;
} ITP_RTPOUT_ch_Property;

#define ITP_SUBFUNC_RTPX_INPUT 0
#define ITP_SUBFUNC_RTPX_OUTPUT 1
#define ITP_SUBFUNC_RTPX_MIXER 2
#define ITP_SUBFUNC_RTPX_ADDR 3
#define ITP_SUBFUNC_RTPX_AUDIO_INPUT 4
#define ITP_SUBFUNC_RTPX_AUDIO_OUTPUT 5
#define ITP_SUBFUNC_RTPX_AUDIO_MIXER 6
#define ITP_SUBFUNC_RTPX_AUDIO_ADDR 7

typedef struct
{
    uint8_t m_u8ChannelEnable;
    uint8_t m_u8MediaType;
    uint8_t m_u8PayloadType;
    uint8_t m_Rfu;
} ITP_RTPXIN_ch_Property;

typedef struct
{
    uint8_t m_u8ChannelEnable;
    uint8_t m_u8MediaType;
    uint8_t m_u8PayloadType;
    uint8_t m_u8SRC_Ctrl;
    uint16_t m_u16SRC_Id;
    uint16_t m_u16Port;
    uint32_t m_u32IP;
} ITP_RTPXOUT_ch_Property;

typedef struct
{
    uint8_t m_u8ChannelEnable;
    uint8_t m_u8MediaType;
    uint8_t m_u8PayloadType;
    uint8_t m_u8Decode;
} ITP_RTPX_AUDIOIN_ch_Property;

typedef struct
{
    uint8_t m_u8ChannelEnable;
    uint8_t m_u8MediaType;
    uint8_t m_u8PayloadType;
    uint8_t m_u8Encode;
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint16_t m_u16SRC1_Id;
    uint16_t m_u16SRC2_Id;
    uint16_t m_u16Port;
    uint32_t m_u32IP;
} ITP_RTPX_AUDIOOUT_ch_Property;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8SRC_Mode;
    uint8_t m_u8SRC1_Ctrl;
    uint8_t m_u8SRC2_Ctrl;
    uint8_t m_u8Encode_Type;
    uint8_t m_u8VAD_Enable;
    uint8_t m_u8Encode;
    uint8_t m_u8AChannelNums;
    uint16_t m_u16SRC1_ID;
    uint16_t m_u16SRC2_ID;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8Stop_Mode;
    uint8_t m_u8Stop_Code;
    uint8_t m_u8Tag_Number_last;
    uint16_t m_u16Stop_Ref_ch;
    uint16_t m_u16Stream_Packet_Len;
    uint8_t m_u8Video_Enable;
    uint8_t m_u8VideoSRC_Mode;
    uint8_t m_u8VideoSRC1_Ctrl;
    uint8_t m_u8VideoSRC2_Ctrl;
    uint8_t m_u8VideoEncode_Type;
    uint8_t m_u8VideoFrameRate;
    uint16_t m_u16VideoWidth;
    uint16_t m_u16VideoHeight;
    uint8_t m_u8Ref[2];
    uint16_t m_u16VideoSRC1_ID;
    uint16_t m_u16VideoSRC2_ID;
    uint8_t m_u8VideoTag_Number;
    uint8_t m_u8VideoStop_Mode;
    uint8_t m_u8VideoStop_Code;
    uint8_t m_u8VideoReserved2;
    uint16_t m_u16VideoStop_Ref_ch;
    uint16_t m_u16VideoStream_Packet_Len;
} ITP_Record_ch_Property_3dot0;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8DTMF_Enable;
    uint8_t m_u8MR2F_Enable;
    uint8_t m_u8MR2B_Enable;
    uint8_t m_u8GTD_Enable;
    uint8_t m_u8FSK_Enable;
    uint8_t m_u8EXTEnable;
    uint8_t m_u8Reserved;
    uint8_t m_u8GTD_ID[8];
} ITP_Gtd_ch_Property;

#define ITP_MAX_GTG_LENGTH 1024
typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8Gtg_Type;
    uint16_t m_u16Code_Length;
    uint8_t m_u8GTG_ID_Buf[ITP_MAX_GTG_LENGTH];
} ITP_Gtg_ch_Property;

typedef struct
{
    uint8_t m_u8SrcType;
    uint8_t m_u8VadTag;
    uint16_t m_u16SrcID;
    uint8_t m_u8InputOpt;
    uint8_t m_u8OutputOpt;
} ITP_Conf_Member_Property;

typedef struct
{
    uint16_t m_u16MemberNum;
    uint16_t m_u16Reserved;
    ITP_Conf_Member_Property confMemberProperty[128];
} ITP_Conf_grp_Operation;

typedef struct
{
    uint8_t m_u8Operation;
    uint8_t m_u8Rfu1[3];
    uint8_t m_u8ResType;
    uint8_t m_u8InputOpt;
    uint8_t m_u8OutputOpt;
    uint8_t m_u8ReportStatus;
    uint16_t m_u16SrcID;
    uint16_t m_u16VadTime;
} ITP_Conf_grp_Property;

typedef struct
{
    uint8_t m_u8VadStates[32];
} ITP_Conf_Vad_Property;

typedef struct
{
    uint32_t m_u32VadLevel;
    uint8_t m_u8Ref[4];
} Vad_Ch_Config;

typedef struct
{
    uint8_t m_u8AgcEnable;
    uint8_t m_u8AgcMode;
    uint8_t m_u8EcEnable;
    uint8_t m_u8EcRefType;
    uint8_t m_u8VadEnable;
    uint8_t m_u8TadEnable;
    uint8_t m_u8NrEnable;
    uint8_t m_u8Reserved;
    uint16_t m_u16FixGain;
    uint16_t m_u16EcRefCh;
    uint32_t m_u32Reserved;
} VocInputControl_t;

typedef struct
{
    uint8_t m_u8AgcEnable;
    uint8_t m_u8AgcMode;
    uint8_t m_u8OutputType;
    uint8_t m_u8Reserved[1];
    uint16_t m_u16FixGain;
    int8_t m_s8Rfu[2];
    MixerControl_t m_MixerControl;
} VocOutputControl_t;

typedef struct
{
    uint8_t m_u8InputCtrlValid;
    uint8_t m_u8PlayGainMode;
    uint16_t m_u16PlayFixGain;
    VocInputControl_t m_VocInputControl;
    uint8_t m_u8OutputCtrlValid;
    int8_t m_s8Rfu2[3];
    VocOutputControl_t m_VocOutputControl;
    uint8_t m_u8GtdCtrlValid;
    int8_t m_s8Rfu3[3];
    ITP_Gtd_ch_Property m_VocGtdControl;
} ITP_Fullvoc_ch_Property;

typedef struct
{
    uint8_t m_u8MediaType;
    uint8_t m_u8Reserve[3];
} ITP_Clear_Play_Buf_CMD;

typedef struct
{
    uint8_t m_u8MediaType;
    uint8_t m_u8Reserved;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8GTD_ID;
} ITP_Play_ch_Event;

typedef struct
{
    uint8_t m_u8data_type;
    uint8_t m_u8Reserved;
    uint8_t m_u8Tag_Number;
    uint8_t m_u8GTD_ID;
} ITP_Record_ch_Event;

typedef struct
{
    uint16_t m_u16GTG_Status;
    uint16_t m_u16Reserved;
} ITP_GTG_ch_Event;

typedef enum
{
    SYN_TYPE_AUDIO_FIRST = 0,
    SYN_TYPE_VIDEO_FIRST = 1
} RECORD_SYN_TYPE;

typedef struct
{
    uint8_t m_u8Syn_Type;
    uint8_t m_u8Rfu[3];
    uint32_t m_u32Syn_time;
} ITP_Record_ch_Syn_Event;

typedef struct
{
    uint32_t m_u32FrameFlag;
    uint16_t m_u16FrameLength;
    uint16_t m_u16FrameDuration;
    uint8_t m_u8FrameEXTType;
    uint8_t m_u8Ref[3];
} ITP_Frame_Data_Head;

//-----------------------------------------------------------------------------
// XX.3 Digital Port Definitions (Func_type=4)
//-----------------------------------------------------------------------------
#define ITP_Analog30 1
#define ITP_E1_SS7_TUP_0_Link 6
#define ITP_E1_SS7_TUP_2_Link 8
#define ITP_T1_D4 9
#define ITP_T1_ESF 10
#define ITP_J1_D4 11
#define ITP_J1_ESF 12
#define ITP_SLC_96 13
#define ITP_E1_SS7_ISUP_0_Link 14
#define ITP_E1_SS7_ISUP_2_Link 16
#define ITP_E1_7D24B 17
#define ITP_E1_1D30B 18
#define ITP_E1_11D20B 19
#define ITP_E1_HIZ_CAS 20
#define ITP_E1_HIZ_PRI 21
#define ITP_E1_HIZ_SS7 22
#define ITP_E1_HIZ_64K_SS7 23
#define ITP_E1_HIZ_2M_SS7 24
#define ITP_E1_LINESIDE 25
#define ITP_E1_HIZ_N64K_HDLC 26
#define ITP_E1_2M_SS7 27

#define ITP_Build_E1_75_AGC 0
#define ITP_Build_E1_120_AGC 1
#define ITP_Build_E1_75_NoAGC 2
#define ITP_Build_E1_120_NoAGC 3

#define ITP_Disable_Tx_G703_Clock 0
#define ITP_Enable_Tx_G703_Clock 1
#define ITP_Disable_Rx_G703_Clock 0
#define ITP_Enable_Rx_G703_Clock 1

#define ITP_Tx_100 2
#define ITP_Rx_100 2

#define ITP_E1_Short_Haul 0
#define ITP_E1_Long_Haul 1
#define ITP_T1_Long_Haul 0
#define ITP_T1_Short_Haul 1

#define ITP_Normal_Operation 0
#define ITP_Boost_20dB 1
#define ITP_Boost_26dB 2
#define ITP_Boost_32dB 3

#define ITP_Depth_128bits 0
#define ITP_Depth_32bits 1
#define ITP_JA_Rx_Side 0
#define ITP_JA_Tx_Side 1

#define ITP_Auto_RAI 1
#define ITP_Auto_AIS 2

#define ITP_E1_CRC4 1
#define ITP_E1_CRC4_Auto 2
#define ITP_T1_J1_CRC6 3

#define ITP_Framer_Loopback 1
#define ITP_Payload_Loopback 2

//-----------------------------------------------------------------------------
// XX.4 Interface Channel Definitions (Func_type=5)
//-----------------------------------------------------------------------------
#define ITP_CH_TYPE_BASE 0x00
#define ITP_CH_TYPE_ANALOG_TRUNK 0x01
#define ITP_CH_TYPE_ANALOG_USER 0x02
#define ITP_CH_TYPE_ANALOG_HIZ 0x19
#define ITP_CH_TYPE_ANALOG_EMPTY 0x04
#define ITP_CH_TYPE_E1_PCM 0x05
#define ITP_CH_TYPE_E1_CAS 0x06
#define ITP_CH_TYPE_E1_PRI 0x07
#define ITP_CH_TYPE_E1_SS7_TUP 0x08
#define ITP_CH_TYPE_E1_SS7_ISUP 0x09
#define ITP_CH_TYPE_ANALOG_VOC2W 0x0A
#define ITP_CH_TYPE_ANALOG_VOC4W 0x0B
#define ITP_CH_TYPE_ANALOG_EM 0x0C
#define ITP_CH_TYPE_ANALOG_MEG 0x0D
#define ITP_CH_TYPE_E1_DCH 0x0E
#define ITP_CH_TYPE_E1_BCH 0x0F
#define ITP_CH_TYPE_UNUSABLE 0x10
#define ITP_CH_TYPE_E1_HIZ_CAS 0x11
#define ITP_CH_TYPE_E1_HIZ_PRI 0x12
#define ITP_CH_TYPE_E1_HIZ_SS7 0x13
#define ITP_CH_TYPE_E1_HIZ_PRI_LINK 0x14
#define ITP_CH_TYPE_E1_HIZ_SS7_64K_LINK 0x15
#define ITP_CH_TYPE_E1_HIZ_SS7_2M_LINK 0x16
#define ITP_CH_TYPE_E1_SS7_LINK 0x17
#define ITP_CH_TYPE_E1_LINESIDE 0x18
#define ITP_CH_TYPE_E1_HIZ_HDLC_N64K_LINK 0x1A
#define ITP_CH_TYPE_E1_HIZ_HDLC 0x1C
#define ITP_CH_TYPE_E1_SS7_2M_LINK 0x1b
#define ITP_CH_TYPE_DIGITAL_RECORD 0x20

#define ITP_ANALOG_TRUNK_ON_HOOK 0x00
#define ITP_ANALOG_TRUNK_OFF_HOOK 0x01
#define ITP_ANALOG_TRUNK_NO_LOOP 0x00
#define ITP_ANALOG_TRUNK_RING_LOOP 0x01
#define ITP_ANALOG_TRUNK_POS_LOOP 0x02
#define ITP_ANALOG_TRUNK_NEG_LOOP 0x03
#define ITP_ANALOG_USER_FEED_POWER 0x00
#define ITP_ANALOG_USER_FEED_RING1 0x01
#define ITP_ANALOG_USER_FEED_RING2 0x02
#define ITP_ANALOG_USER_FEED_RING3 0x03
#define ITP_ANALOG_USER_NO_LOOP 0x00
#define ITP_ANALOG_USER_LOOP_CUR 0x01

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8Type;
    uint8_t m_u8Tx_State;
    uint8_t m_u8Rx_State;
} ITP_Interface_ch_Property;

typedef struct
{
    uint8_t m_u8UpDateflag;
    uint8_t m_u8ChId;
    uint8_t m_u8Voltage;
    uint8_t reserved;
} ITP_Interface_ch_Set_Voltage;

typedef struct
{
    uint8_t m_u8UpDateflag;
    uint8_t m_u8ChId;
    int8_t m_s8Voltage;
    uint8_t reserved[5];
} ITP_Interface_ch_Read_Voltage;

typedef struct
{
    uint8_t m_u8UnitId;
    uint8_t m_u8SN[13];
    uint8_t reserved[2];
} ITP_Interface_ch_Read_Sn;

typedef struct
{
    int8_t m_u8Serial_Write[13];
    int8_t m_u8Serial_Read[13];
    uint8_t m_u8Reserved;
} ITP_Interface_Digital_Module_Serial;

typedef struct
{
    uint8_t m_u8State;
    uint8_t m_u8Reserved;
} ITP_Res_State;

typedef struct
{
    uint8_t m_u8HandOnOffFlag;
    uint8_t m_u8CallInOutFlag;
    uint8_t m_u8CallerNumber_Lenth;
    int8_t m_u8CallerNoumber[16];
    uint8_t m_u8Reserved[4];
} ITP_Interface_Digital_Rec_Event;

typedef struct
{
    uint16_t m_u16DataLen;
    uint8_t m_u8Reserved[2];
    int16_t m_s16Databuf[1024];
} ITP_Interface_Digital_Rec_DData;

typedef struct
{
    uint8_t m_u8Channel_Enable;
    uint8_t m_u8Exchange_Type;
    uint8_t m_u8Encode_Type;
    uint16_t m_u8Analog_agc;
    uint8_t m_u8Reserved;
} ITP_Interface_Digital_Rec_Property;

typedef struct tagFSK_RX_EVENT
{
    int16_t m_u16FSK_Length;
    uint8_t m_u8FSK_ID_Buf[32];
} FSK_RX_EVENT;

typedef struct
{
    uint8_t m_u8Len;
    uint8_t m_u8ModuleNum;
    uint8_t m_u8StartAddr;
    uint8_t m_u8Reserved;
} ITP_T_READ_INTERFACE_CH_SNR;

typedef struct
{
    uint8_t m_u8Len;
    uint8_t m_u8ModuleNum;
    uint8_t m_u8StartAddr;
    uint8_t m_u8Data[17];
} ITP_T_INTERFACE_CH_SNR;

typedef struct
{
    uint8_t m_u8Len;
    uint8_t m_u8Data[19];
} ITP_T_INTERFACE_DJ01_SNR;

typedef struct
{
    uint8_t m_u8Type;
    uint8_t m_u8Reserved[3];
} ITP_T_INTERFACE_CH_TYPE;

typedef struct
{
    uint8_t m_u8Voltage;
    uint8_t m_u8Reserved[3];
} ITP_T_INTERFACE_CH_VOLTAGE;

//-----------------------------------------------------------------------------
// XX.5 DSS1 Link Definitions (Func_type=6)
//-----------------------------------------------------------------------------
#define ITP_SUBFUNC_SET_DSS1_LINK_CONFIG 0x01
#define ITP_SUBFUNC_GET_DSS1_LINK_CONFIG 0x02
#define ITP_SUBFUNC_DSS1_LINK_CONTROL 0x03
#define ITP_SUBFUNC_DSS1_LINK_SEND_DATA 0x04
#define ITP_SUBFUNC_DSS1_LINK_CONFIG 0x02
#define ITP_SUBFUNC_DSS1_LINK_EVENT 0x03
#define ITP_SUBFUNC_DSS1_LINK_REV_DATA 0x04
#define ITP_SUBFUNC_DSS1_LINK_REQ_SEND 0x05
#define ITP_SUBFUNC_DSS1_LINK_DEBUG_DATA 0x06

#define ITP_DATA_TYPE_I 0
#define ITP_DATA_TYPE_UI 1

typedef struct
{
    uint16_t m_u16remain_space;
    uint8_t m_u8data_type;
    uint8_t m_u8Reserved[1];
} ITP_DSS1_link_Remain_space;

//-----------------------------------------------------------------------------
// XX.6 SS7 Link Definitions (Func_type=7)
//-----------------------------------------------------------------------------
#define ITP_SUBFUNC_SET_SS7_LINK_CONFIG 0x01
#define ITP_SUBFUNC_GET_SS7_LINK_CONFIG 0x02
#define ITP_SUBFUNC_SS7_LINK_CONTROL 0x03
#define ITP_SUBFUNC_SS7_LINK_SEND_DATA 0x04
#define ITP_SUBFUNC_SS7_LINK_CONFIG 0x02
#define ITP_SUBFUNC_SS7_LINK_EVENT 0x03
#define ITP_SUBFUNC_SS7_LINK_REV_DATA 0x04
#define ITP_SUBFUNC_SS7_LINK_REQ_SEND 0x05
#define ITP_SUBFUNC_SS7_LINK_DEBUG_DATA 0x06
#define ITP_SUBFUNC_SS7_LINK_CHANGEOVER_DATA 0x07

#define ITP_SS7_LINK_COMMAND_START 0x01
#define ITP_SS7_LINK_COMMAND_STOP 0x02
#define ITP_SS7_LINK_COMMAND_EMERGENCY_START 0x04
#define ITP_SS7_LINK_COMMAND_EMERGENCY_STOP 0x19
#define ITP_SS7_LINK_COMMAND_RECOVER_BSNT 0x05
#define ITP_SS7_LINK_COMMAND_RECOVER_REQ_FSNC 0x06
#define ITP_SS7_LINK_COMMAND_LOCAL_PROC_OUT 0x17
#define ITP_SS7_LINK_COMMAND_LOCAL_PROC_WELL 0x09
#define ITP_SS7_LINK_COMMAND_L3_ERROR 0x07

#define ITP_SS7_LINK_STATE_IN_SERVICE 0x01
#define ITP_SS7_LINK_STATE_OUT_OF_SERVICE 0x02
#define ITP_SS7_LINK_STATE_REMOTE_PROC_OUT 0x03
#define ITP_SS7_LINK_STATE_REMOTE_PROC_WELL 0x04
#define ITP_SS7_LINK_STATE_GIVE_RBSNT 0x77
#define ITP_SS7_LINK_STATE_RESTORE_OVER 0x78
#define ITP_SS7_LINK_STATE_RESTORE 0x79
#define ITP_SS7_LINK_STATE_RESTORE_ERROR 0x7A

#define ITP_SS7_DATA_TYPE_NORMAL_MSU 0
#define ITP_SS7_DATA_TYPE_RESTORE_MSU 1

typedef struct
{
    uint8_t m_u8Debug_Enable;
    uint8_t m_u8ss7_link_state;
    uint8_t m_u8Reserved[2];
    uint16_t m_u16Debug_Interval;
    uint16_t m_u16ts_index;
} ITP_SS7_link_Property;

typedef struct
{
    uint8_t m_u8ss7_link_command;
    uint8_t m_u8fsnc;
    uint8_t m_u8Reserved[2];
} ITP_SS7_link_Command;

typedef struct
{
    uint8_t m_u8ss7_link_state;
    uint8_t m_u8bsnt;
    uint8_t m_u8Reserved[2];
} ITP_SS7_link_State;

typedef struct
{
    uint16_t m_u16data_length;
    uint16_t m_u16data_type;
    uint8_t m_u8send_data[ITP_MAX_LINK_DATA_LENGTH];
} ITP_SS7_link_Send_Data;

typedef struct
{
    uint16_t m_u16data_length;
    uint16_t m_u16data_type;
    uint8_t m_u8rev_data[ITP_MAX_LINK_DATA_LENGTH];
} ITP_SS7_link_Rev_Data;

typedef struct
{
    uint16_t m_u16data_length;
    uint16_t m_u16data_type;
    uint8_t m_u8debug_data[ITP_MAX_LINK_DEBUG_DATA_LENGTH];
} ITP_SS7_link_debug_Data;

typedef struct
{
    uint32_t m_u32remain_space;
} ITP_SS7_link_Send_Request;

//-----------------------------------------------------------------------------
// XX.7 CT-BUS Clock Definitions (Func_type=8)
//-----------------------------------------------------------------------------
#define ITP_SUBFUNC_SET_CTBUS_CLK_CONFIG 0x01
#define ITP_SUBFUNC_GET_CTBUS_CLK_CONFIG 0x02
#define ITP_SUBFUNC_SET_CTBUS_CLK_NETREF 0x03
#define ITP_SUBFUNC_GET_CTBUS_CLK_STATE 0x04
#define ITP_SUBFUNC_CTBUS_CLK_CONFIG 0x02
#define ITP_SUBFUNC_CTBUS_CLK_STATE 0x04

#define ITP_CTBUS_CLK_MASTER_A 0x00
#define ITP_CTBUS_CLK_MASTER_B 0x01
#define ITP_CTBUS_CLK_PRIMARY_MASTER 0x00
#define ITP_CTBUS_CLK_SECONDARY_MASTER 0x01
#define ITP_CTBUS_CLK_MASTER_PLL_NORMAL 0x00
#define ITP_CTBUS_CLK_MASTER_PLL_HOLDOVER 0x02
#define ITP_CTBUS_CLK_MASTER_PLL_FREE_RUN 0x03
#define ITP_CTBUS_CLK_MASTER_PLL_AUTO_HOLDOVER 0x06
#define ITP_CTBUS_CLK_MASTER_PLL_AUTO_FREE_RUN 0x07
#define ITP_CTBUS_CLK_MASTER_PLL_REF_NET1 0x06
#define ITP_CTBUS_CLK_MASTER_PLL_REF_NET2 0x07
#define ITP_CTBUS_CLK_NET_SOURCE_FROM_L0 0x8
#define ITP_CTBUS_CLK_NET_SOURCE_FROM_L1 0x9
#define ITP_CTBUS_CLK_NET_SOURCE_FROM_L2 0xA
#define ITP_CTBUS_CLK_NET_SOURCE_FROM_L3 0xB

typedef struct
{
    uint8_t m_u8Master_Enable;
    uint8_t m_u8A_B_Select;
    uint8_t m_u8Advance_Timing;
    uint8_t m_u8Auto_Switch;
    uint8_t m_u8Primary_mode;
    uint8_t m_u8Master_PLL;
    uint8_t m_u8MTIE_Compatible;
    uint8_t m_u8PLL_Reference;
    uint8_t m_u8CT_NET1_Enable;
    uint8_t m_u8CT_NET1_Source;
    uint8_t m_u8CT_NET2_Enable;
    uint8_t m_u8CT_NET2_Source;
} ITP_CTBUS_clk_Property;

typedef struct
{
    uint8_t m_u8CTMaster_Enable;
    uint8_t m_u8CTMaster_Pri_Sec_Readback;
    uint8_t m_u8CTMaster_PLL_Mode_Readback;
    uint8_t m_u8CTSlave_ABSelect_Readback;
    uint8_t m_u8CTNetRef1_Output_Enable;
    uint8_t m_u8CTNetRef1_Source;
    uint8_t m_u8CTNetRef2_Output_Enable;
    uint8_t m_u8CTNetRef2_Source;
    uint8_t m_u8CTError_Latch;
    uint8_t m_u8Reserved[3];
} ITP_CTBUS_clk_State;

//-----------------------------------------------------------------------------
// XX.8 CT-BUS Timeslot Definitions (Func_type=A)
//-----------------------------------------------------------------------------
#define ITP_SUBFUNC_LINK_TIMESLOT 0x01
#define ITP_SUBFUNC_UNLINK_TIMESLOT 0x02
#define ITP_SUBFUNC_DUALLINK_TIMESLOT 0x03
#define ITP_SUBFUNC_DUALUNLINK_TIMESLOT 0x04

#define ITP_TS_TYPE_VOICE_CH 0x02
#define ITP_TS_TYPE_INTERFACE_CH 0x05
#define ITP_TS_TYPE_CTBUS_TS 0x09
#define ITP_TS_TYPE_EXCHANGE_TS 0x10

typedef struct
{
    uint8_t m_u8src_type;
    uint8_t m_u8dst_type;
    uint8_t m_u8Reserved[2];
    uint16_t m_u16src_ch_index;
    uint16_t m_u16dst_ch_index;
} ITP_Connect_ts_Property;

#define ITP_MAX_INIT_PARA_LEN 24
#define ITP_MAX_INIT_PARA_NUMBER 5
typedef struct
{
    uint16_t m_u16para_type;
    uint16_t m_u16para_len;
    uint8_t m_u8para_data[ITP_MAX_INIT_PARA_LEN];
} ITP_Init_Para_Data;

typedef struct
{
    uint16_t m_u16total_para_number;
    uint16_t m_u16Reserved;
    ITP_Init_Para_Data m_ITP_init_para_data[ITP_MAX_INIT_PARA_NUMBER];
} ITP_Init_Para;

typedef struct
{
    uint8_t m_u8Module_Name[8];
    uint32_t m_u32Ver_Number;
    uint16_t m_u16Res_List_Length;
    uint16_t m_u16Dsp_Index;
    ITP_Res_Cap m_ITP_resource_capacity[ITP_MAX_RES_LIST_LENGTH];
} ITP_T_DSP_INIT_STATUS;

//-----------------------------------------------------------------------------
// XX.9 Misc Definitions
//-----------------------------------------------------------------------------
#define OK_STORED_INI_INFO 1
#define ERROR_MODULE_TYPE_INI_INFO 1001
#define ERROR_VERSION_INI_INFO 1002
#define ERROR_LEN_INI_INFO 1003
#define ERROR_STORE_INI_INFO 1004
#define NONE_VALID_LICENSE 1005

#define FACE_MAIN_FUNCTION_EXPORT 0x4
#define FACE_MAIN_FUNCTION_IMPORT 0x5
#define FACE_MAIN_FUNCTION_LICENSE_IMPORT 0x6

#define ITP_DATATYPE_NULL 0x0
#define ITP_DATATYPE_AUDIO 0x01
#define ITP_DATATYPE_VIDEO 0x02

typedef struct
{
    uint8_t Master_Enable;
    uint8_t A_B_Select;
    uint8_t Advance_Timing;
    uint8_t Auto_Switch;
    uint8_t Primary_mode;
    uint8_t Master_PLL;
    uint8_t MTIE_Compatible;
    uint8_t PLL_Reference;
    uint8_t CT_NET1_Enable;
    uint8_t CT_NET1_Source;
    uint8_t CT_NET2_Enable;
    uint8_t CT_NET2_Source;
} TCTBUS_clk_Property;

#define SIGNALTONE_DIAL_CH ('G')
#define SIGNALTONE_RTONE_CH ('H')
#define SIGNALTONE_RTONE1_CH ('h')
#define SIGNALTONE_BUSY1_CH ('I')
#define SIGNALTONE_BUSY2_CH ('J')
#define SIGNALTONE_BUSY3_CH ('K')
#define SIGNALTONE_BROKEN_CH ('b')

#endif // DONJIN_DSP_DEF_H
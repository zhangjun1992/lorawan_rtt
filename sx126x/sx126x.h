/*!
 * \file      sx126x.h
 *
 * \brief     SX126x driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SX126x_H__
#define __SX126x_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "gpio.h"
#include "spi.h"
#include "radio.h"

#define SX1261                                      1
#define SX1262                                      2

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME                           3 // [ms]

/*!
 * \brief Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
 */
#define AUTO_RX_TX_OFFSET                           2

/*!
 * \brief LFSR initial value to compute IBM type CRC
 */
#define CRC_IBM_SEED                                0xFFFF

/*!
 * \brief LFSR initial value to compute CCIT type CRC
 */
#define CRC_CCITT_SEED                              0x1D0F

/*!
 * \brief Polynomial used to compute IBM CRC
 */
#define CRC_POLYNOMIAL_IBM                          0x8005

/*!
 * \brief Polynomial used to compute CCIT CRC
 */
#define CRC_POLYNOMIAL_CCITT                        0x1021

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 */
#define REG_LR_CRCSEEDBASEADDR                      0x06BC

/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRCPOLYBASEADDR                      0x06BE

/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9

/*!
 * \brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKETPARAMS                         0x0704

/*!
 * \brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOADLENGTH                        0x0702

/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNCWORDBASEADDRESS                  0x06C0

/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD                             0x0740

/*!
 * Syncword for Private LoRa networks  私有网络同步字
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x1424

/*!
 * Syncword for Public LoRa networks  公有网络同步字
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x3444

/*!
 * The address of the register giving a 4 bytes random number
 */
#define RANDOM_NUMBER_GENERATORBASEADDR             0x0819

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN                                 0x08AC

/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM                                0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP                                     0x08E7

/*!
 * \brief Structure describing the radio status  radio 状态描述结构体
 */
typedef union RadioStatus_u
{
    uint8_t Value;
    struct
    {   //bit order is lsb -> msb
        uint8_t Reserved  : 1;  //!< Reserved //保留
        uint8_t CmdStatus : 3;  //!< Command status //命令状态
        uint8_t ChipMode  : 3;  //!< Chip mode  //芯片模式
        uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy cpu繁忙
    }Fields;
}RadioStatus_t;

/*!
 * \brief Structure describing the error codes for callback functions  回调函数错误代码定义枚举
 */
typedef enum
{
    IRQ_HEADER_ERROR_CODE                   = 0x01, //头部错误
    IRQ_SYNCWORD_ERROR_CODE                 = 0x02, //同步子错误
    IRQ_CRC_ERROR_CODE                      = 0x04  //CRC校验错误
}IrqErrorCode_t;

enum IrqPblSyncHeaderCode_t
{
    IRQ_PBL_DETECT_CODE                     = 0x01,//
    IRQ_SYNCWORD_VALID_CODE                 = 0x02,//同步字有效
    IRQ_HEADER_VALID_CODE                   = 0x04//头部有效
};

/*!
 * \brief Represents the operating mode the radio is actually running  实际正在运行的模式
 */
typedef enum
{
    MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode 睡眠
    MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator RC待机模式
    MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator 晶振待机模式
    MODE_FS,                                                //! The radio is in frequency synthesis mode 频率合成模式
    MODE_TX,                                                //! The radio is in transmit mode 发送数据
    MODE_RX,                                                //! The radio is in receive mode 接收数据
    MODE_RX_DC,                                             //! The radio is in receive duty cycle mode 占空比接收模式
    MODE_CAD                                                //! The radio is in channel activity detection mode  CAD检测 信道活跃检测
}RadioOperatingModes_t;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *        在待机模式下，使用的晶振模式
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications  xocs 温补晶振，针对对时间有严格要求的场合
 */
typedef enum
{
    STDBY_RC                                = 0x00,
    STDBY_XOSC                              = 0x01
}RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    USE_LDO                                 = 0x00, // default 
    USE_DCDC                                = 0x01
}RadioRegulatorMode_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    PACKET_TYPE_GFSK                        = 0x00,//GFSK
    PACKET_TYPE_LORA                        = 0x01,//LORA
    PACKET_TYPE_NONE                        = 0x0F
}RadioPacketTypes_t;

/*!
 * \brief Represents the ramping time for power amplifier 功率放大器斜波时间
 */
typedef enum
{
    RADIO_RAMP_10_US                        = 0x00,
    RADIO_RAMP_20_US                        = 0x01,
    RADIO_RAMP_40_US                        = 0x02,
    RADIO_RAMP_80_US                        = 0x03,
    RADIO_RAMP_200_US                       = 0x04,
    RADIO_RAMP_800_US                       = 0x05,
    RADIO_RAMP_1700_US                      = 0x06,
    RADIO_RAMP_3400_US                      = 0x07
}RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 *          通道活跃检测符号数
 */
typedef enum
{
    LORA_CAD_01_SYMBOL                      = 0x00,
    LORA_CAD_02_SYMBOL                      = 0x01,
    LORA_CAD_04_SYMBOL                      = 0x02,
    LORA_CAD_08_SYMBOL                      = 0x03,
    LORA_CAD_16_SYMBOL                      = 0x04
}RadioLoRaCadSymbols_t;

/*!
 * \brief Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum
{
    LORA_CAD_ONLY                           = 0x00,
    LORA_CAD_RX                             = 0x01,  //CAD 接收
    LORA_CAD_LBT                            = 0x10   //LBT  发送前监听
}RadioCadExitModes_t;//radio cad 退出模式

/*!
 * \brief Represents the modulation shaping parameter   //？
 */
typedef enum
{
    MOD_SHAPING_OFF                         = 0x00,
    MOD_SHAPING_G_BT_03                     = 0x08,
    MOD_SHAPING_G_BT_05                     = 0x09,
    MOD_SHAPING_G_BT_07                     = 0x0A,
    MOD_SHAPING_G_BT_1                      = 0x0B
}RadioModShapings_t;

/*!
 * \brief Represents the modulation shaping parameter  
 */
typedef enum
{
    RX_BW_4800                              = 0x1F,
    RX_BW_5800                              = 0x17,
    RX_BW_7300                              = 0x0F,
    RX_BW_9700                              = 0x1E,
    RX_BW_11700                             = 0x16,
    RX_BW_14600                             = 0x0E,
    RX_BW_19500                             = 0x1D,
    RX_BW_23400                             = 0x15,
    RX_BW_29300                             = 0x0D,
    RX_BW_39000                             = 0x1C,
    RX_BW_46900                             = 0x14,
    RX_BW_58600                             = 0x0C,
    RX_BW_78200                             = 0x1B,
    RX_BW_93800                             = 0x13,
    RX_BW_117300                            = 0x0B,
    RX_BW_156200                            = 0x1A,
    RX_BW_187200                            = 0x12,
    RX_BW_234300                            = 0x0A,
    RX_BW_312000                            = 0x19,
    RX_BW_373600                            = 0x11,
    RX_BW_467000                            = 0x09
}RadioRxBandwidth_t;  //接收带宽参数

/*!
 * \brief Represents the possible spreading factor values in LoRa packet types
 *      扩频因子
 */
typedef enum
{
    LORA_SF5                                = 0x05,
    LORA_SF6                                = 0x06,
    LORA_SF7                                = 0x07,
    LORA_SF8                                = 0x08,
    LORA_SF9                                = 0x09,
    LORA_SF10                               = 0x0A,
    LORA_SF11                               = 0x0B,
    LORA_SF12                               = 0x0C
}RadioLoRaSpreadingFactors_t;

/*!
 * \brief Represents the bandwidth values for LoRa packet type
 *    带宽
 */
typedef enum
{
    LORA_BW_500                             = 6,
    LORA_BW_250                             = 5,
    LORA_BW_125                             = 4,
    LORA_BW_062                             = 3,
    LORA_BW_041                             = 10,
    LORA_BW_031                             = 2,
    LORA_BW_020                             = 9,
    LORA_BW_015                             = 1,
    LORA_BW_010                             = 8,
    LORA_BW_007                             = 0
}RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LoRa packet type
 *       编码率
 */
typedef enum
{
    LORA_CR_4_5                             = 0x01,
    LORA_CR_4_6                             = 0x02,
    LORA_CR_4_7                             = 0x03,
    LORA_CR_4_8                             = 0x04
}RadioLoRaCodingRates_t;

/*!
 * \brief Represents the preamble length used to detect the packet on Rx side
 *      前序探测字节数          
 */
typedef enum
{
    RADIO_PREAMBLE_DETECTOR_OFF             = 0x00,         //!< Preamble detection length off
    RADIO_PREAMBLE_DETECTOR_08_BITS         = 0x04,         //!< Preamble detection length 8 bits
    RADIO_PREAMBLE_DETECTOR_16_BITS         = 0x05,         //!< Preamble detection length 16 bits
    RADIO_PREAMBLE_DETECTOR_24_BITS         = 0x06,         //!< Preamble detection length 24 bits
    RADIO_PREAMBLE_DETECTOR_32_BITS         = 0x07         //!< Preamble detection length 32 bit
}RadioPreambleDetection_t;

/*!
 * \brief Represents the possible combinations of SyncWord correlators activated
 * 同步子可能的组合
 */
typedef enum
{
    RADIO_ADDRESSCOMP_FILT_OFF              = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
    RADIO_ADDRESSCOMP_FILT_NODE             = 0x01,
    RADIO_ADDRESSCOMP_FILT_NODE_BROAD       = 0x02
}RadioAddressComp_t;

/*!
 *  \brief Radio GFSK packet length mode  GFSK包长度模式，
 */
typedef enum
{
    RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< 固定The packet is known on both sides, no header included in the packet
    RADIO_PACKET_VARIABLE_LENGTH            = 0x01         //!< 可变The packet is on variable size, header included
}RadioPacketLengthModes_t;

/*!
 * \brief Represents the CRC length CRC模式
 */
typedef enum
{
    RADIO_CRC_OFF                           = 0x01,         //!< No CRC in use
    RADIO_CRC_1_BYTES                       = 0x00,
    RADIO_CRC_2_BYTES                       = 0x02,
    RADIO_CRC_1_BYTES_INV                   = 0x04,
    RADIO_CRC_2_BYTES_INV                   = 0x06,
    RADIO_CRC_2_BYTES_IBM                   = 0xF1,
    RADIO_CRC_2_BYTES_CCIT                  = 0xF2
}RadioCrcTypes_t;

/*!
 * \brief Radio whitening mode activated or deactivated  ？？没明白啥意思
 */
typedef enum
{
    RADIO_DC_FREE_OFF                       = 0x00,
    RADIO_DC_FREEWHITENING                  = 0x01
}RadioDcFree_t;

/*!
 * \brief Holds the Radio lengths mode for the LoRa packet type
 */
typedef enum
{
    LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    LORA_PACKET_FIXED_LENGTH                = 0x01,         //!< The packet is known on both sides, no header included in the packet
    LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
    LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH
}RadioLoRaPacketLengthsMode_t;

/*!
 * \brief Represents the CRC mode for LoRa packet type
 */
typedef enum
{
    LORA_CRC_ON                             = 0x01,         //!< CRC activated  打开CRC
    LORA_CRC_OFF                            = 0x00         //!< CRC not used
}RadioLoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LoRa packet type 
 */
typedef enum
{
    LORA_IQ_NORMAL                          = 0x00,
    LORA_IQ_INVERTED                        = 0x01   //IQ反转
}RadioLoRaIQModes_t;

/*!
 * \brief Represents the voltage used to control the TCXO on/off from DIO3  DIO3控制 TCXO的电压
 */
typedef enum
{
    TCXO_CTRL_1_6V                          = 0x00,
    TCXO_CTRL_1_7V                          = 0x01,
    TCXO_CTRL_1_8V                          = 0x02,
    TCXO_CTRL_2_2V                          = 0x03,
    TCXO_CTRL_2_4V                          = 0x04,
    TCXO_CTRL_2_7V                          = 0x05,
    TCXO_CTRL_3_0V                          = 0x06,
    TCXO_CTRL_3_3V                          = 0x07
}RadioTcxoCtrlVoltage_t;

/*!
 * \brief Represents the interruption masks available for the radio  radio可用的中断掩码
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
    IRQ_RADIO_NONE                          = 0x0000,
    IRQ_TX_DONE                             = 0x0001, //发送完成
    IRQ_RX_DONE                             = 0x0002, //接收完成
    IRQ_PREAMBLE_DETECTED                   = 0x0004, //前导码检测
    IRQ_SYNCWORD_VALID                      = 0x0008, //同步字有效
    IRQ_HEADER_VALID                        = 0x0010, //头有效
    IRQ_HEADER_ERROR                        = 0x0020, //头错误
    IRQ_CRC_ERROR                           = 0x0040, //crc错误
    IRQ_CAD_DONE                            = 0x0080, //cad完成
    IRQ_CAD_ACTIVITY_DETECTED               = 0x0100, //car 活跃检测
    IRQ_RX_TX_TIMEOUT                       = 0x0200, //发送、接收超时
    IRQ_RADIO_ALL                           = 0xFFFF
}RadioIrqMasks_t;

/*!
 * \brief Represents all possible opcode understood by the radio 操作命令
 */
typedef enum RadioCommands_e
{
    RADIO_GET_STATUS                        = 0xC0, //获取状态
    RADIO_WRITE_REGISTER                    = 0x0D, //写寄存器
    RADIO_READ_REGISTER                     = 0x1D, //读寄存器
    RADIO_WRITE_BUFFER                      = 0x0E, //写缓冲区
    RADIO_READ_BUFFER                       = 0x1E, //读缓冲区
    RADIO_SET_SLEEP                         = 0x84, //设置睡眠
    RADIO_SET_STANDBY                       = 0x80, //设置待机
    RADIO_SET_FS                            = 0xC1, //设置频率合成
    RADIO_SET_TX                            = 0x83, //设置发送
    RADIO_SET_RX                            = 0x82, //设置接收
    RADIO_SET_RXDUTYCYCLE                   = 0x94, //设置占空比接收
    RADIO_SET_CAD                           = 0xC5, //设置cad
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1, //发送连续波，用于功率测试
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2, //
    RADIO_SET_PACKETTYPE                    = 0x8A, //设置包类型
    RADIO_GET_PACKETTYPE                    = 0x11, //获取包类型
    RADIO_SET_RFFREQUENCY                   = 0x86, //设置RF frequency 射频频率
    RADIO_SET_TXPARAMS                      = 0x8E, //设置发送参数
    RADIO_SET_PACONFIG                      = 0x95, //设置PA配置
    RADIO_SET_CADPARAMS                     = 0x88, //设置cad 参数
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F, //缓存基地址
    RADIO_SET_MODULATIONPARAMS              = 0x8B, //modulation params 设置调制参数
    RADIO_SET_PACKETPARAMS                  = 0x8C, //设置包参数
    RADIO_GET_RXBUFFERSTATUS                = 0x13, //获取接收缓存状态
    RADIO_GET_PACKETSTATUS                  = 0x14, //或者包状态
    RADIO_GET_RSSIINST                      = 0x15, //获取RSSI 
    RADIO_GET_STATS                         = 0x10, //获取统计
    RADIO_RESET_STATS                       = 0x00, //复位统计
    RADIO_CFG_DIOIRQ                        = 0x08, //获取DIO中断
    RADIO_GET_IRQSTATUS                     = 0x12, //获取中断状态
    RADIO_CLR_IRQSTATUS                     = 0x02, //清楚中断状态
    RADIO_CALIBRATE                         = 0x89, //
    RADIO_CALIBRATEIMAGE                    = 0x98,
    RADIO_SET_REGULATORMODE                 = 0x96, //监管模式？
    RADIO_GET_ERROR                         = 0x17, //获取错误
    RADIO_CLR_ERROR                         = 0x07, //清除错误
    RADIO_SET_TCXOMODE                      = 0x97, //温补晶振模式
    RADIO_SET_TXFALLBACKMODE                = 0x93, //回退模式？
    RADIO_SET_RFSWITCHMODE                  = 0x9D, //射频开关
    RADIO_SET_STOPRXTIMERONPREAMBLE         = 0x9F, //停止时间循环 
    RADIO_SET_LORASYMBTIMEOUT               = 0xA0  //lora syme 超时
}RadioCommands_t;

/*!
 * \brief The type describing the modulation parameters for every packet types  对每个包类型的调制参数描述
 */
typedef struct
{
    RadioPacketTypes_t                   PacketType;        //包类型!< Packet to which the modulation parameters are referring to.
    struct
    {
        struct
        {
            uint32_t                     BitRate; //比特率
            uint32_t                     Fdev;    //
            RadioModShapings_t           ModulationShaping;//调制形成
            uint8_t                      Bandwidth; //带宽
        }Gfsk;
        struct
        {
            RadioLoRaSpreadingFactors_t  SpreadingFactor;   //!< Spreading Factor for the LoRa modulation 传播系数，是不是扩频因子？
            RadioLoRaBandwidths_t        Bandwidth;         //!< Bandwidth for the LoRa modulation  带宽
            RadioLoRaCodingRates_t       CodingRate;        //!< Coding rate for the LoRa modulation 编码率
            uint8_t                      LowDatarateOptimize; //!< Indicates if the modem uses the low datarate optimization 低速率优化
        }LoRa;
    }Params;                                                //!< Holds the modulation parameters structure
}ModulationParams_t;

/*!
 * \brief The type describing the packet parameters for every packet types 描述每一个包类型的包参数
 */
typedef struct
{
    RadioPacketTypes_t                    PacketType;        //包类型!< Packet to which the packet parameters are referring to.
    struct
    {
        /*!
         * \brief Holds the GFSK packet parameters
         */
        struct
        {
            uint16_t                     PreambleLength;    //!< The preamble Tx length for GFSK packet type in bit 前导码长度 gfsk
            RadioPreambleDetection_t     PreambleMinDetect; //!< The preamble Rx length minimal for GFSK packet type 
            uint8_t                      SyncWordLength;    //!< The synchronization word length for GFSK packet type gfsk 同步字长度
            RadioAddressComp_t           AddrComp;          //!< Activated SyncWord correlators 
            RadioPacketLengthModes_t     HeaderType;        //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
            uint8_t                      PayloadLength;     //!< Size of the payload in the GFSK packet
            RadioCrcTypes_t              CrcLength;         //!< Size of the CRC block in the GFSK packet
            RadioDcFree_t                DcFree;
        }Gfsk;
        /*!
         * \brief Holds the LoRa packet parameters
         */
        struct
        {
            uint16_t                     PreambleLength;    //!< The preamble length is the number of LoRa symbols in the preamble
            RadioLoRaPacketLengthsMode_t HeaderType;        //!< If the header is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
            uint8_t                      PayloadLength;     //!< Size of the payload in the LoRa packet
            RadioLoRaCrcModes_t          CrcMode;           //!< Size of CRC block in LoRa packet
            RadioLoRaIQModes_t           InvertIQ;          //!< Allows to swap IQ for LoRa packet
        }LoRa;
    }Params;                                                //!< Holds the packet parameters structure
}PacketParams_t;

/*!
 * \brief Represents the packet status for every packet type 每一个包的包状态
 */
typedef struct
{
    RadioPacketTypes_t                    packetType;      //!< Packet to which the packet status are referring to.
    struct
    {
        struct
        {
            uint8_t RxStatus;  //接收窗体
            int8_t RssiAvg;                                //!< The averaged RSSI  RSSI平均值
            int8_t RssiSync;                               //!< The RSSI measured on last packet 最后一个包测量的RSSI
            uint32_t FreqError; //频率错误
        }Gfsk; //GFSK
        struct
        {
            int8_t RssiPkt;                                //!< The RSSI of the last packet
            int8_t SnrPkt;                                 //!< The SNR of the last packet 最后一个包的信噪比
            int8_t SignalRssiPkt; //信号强度 最后一个包？
            uint32_t FreqError;//频率错误
        }LoRa;
    }Params;
}PacketStatus_t;

/*!
 * \brief Represents the Rx internal counters values when GFSK or LoRa packet type is used 接收内部计数器值
 */
typedef struct
{
    RadioPacketTypes_t                    packetType;       //!< Packet to which the packet status are referring to.
    uint16_t PacketReceived;//已接收包
    uint16_t CrcOk; //crcok数
    uint16_t LengthError; //长度错误数
}RxCounter_t;

/*!
 * \brief Represents a calibration configuration 校准配置
 */
typedef union
{
    struct
    {
        uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock RC64K时钟使能校准
        uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock RC13M时钟校准
        uint8_t PLLEnable      : 1;                             //!< Calibrate PLL  锁相环使能
        uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse  adc脉冲
        uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
        uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
        uint8_t ImgEnable      : 1;
        uint8_t                : 1;
    }Fields;
    uint8_t Value;
}CalibrationParams_t;//校准参数

/*!
 * \brief Represents a sleep mode configuration 睡眠模式配置
 */
typedef union
{
    struct
    {
        uint8_t WakeUpRTC               : 1;  //!< Get out of sleep mode if wakeup signal received from RTC
        uint8_t Reset                   : 1; //复位
        uint8_t WarmStart               : 1; //热启动
        uint8_t Reserved                : 5;
    }Fields;
    uint8_t Value;
}SleepParams_t;

/*!
 * \brief Represents the possible radio system error states
 */
typedef union
{
    struct
    {
        uint8_t Rc64kCalib              : 1;                    //!< RC 64kHz oscillator calibration failed
        uint8_t Rc13mCalib              : 1;                    //!< RC 13MHz oscillator calibration failed
        uint8_t PllCalib                : 1;                    //!< PLL calibration failed
        uint8_t AdcCalib                : 1;                    //!< ADC calibration failed
        uint8_t ImgCalib                : 1;                    //!< Image calibration failed
        uint8_t XoscStart               : 1;                    //!< XOSC oscillator failed to start
        uint8_t PllLock                 : 1;                    //!< PLL lock failed
        uint8_t BuckStart               : 1;                    //!< Buck converter failed to start
        uint8_t PaRamp                  : 1;                    //!< PA ramp failed
        uint8_t                         : 7;                    //!< Reserved
    }Fields;
    uint16_t Value;
}RadioError_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct SX126x_s
{
    Gpio_t        Reset;//复位脚
    Gpio_t        BUSY;//繁忙检测脚
    Gpio_t        DIO1;//DIO1
    Gpio_t        DIO2;//DIO2
    Gpio_t        DIO3;//DIO3
    Spi_t         Spi; //spi
    PacketParams_t PacketParams;//包参数
    PacketStatus_t PacketStatus;//包状态
    ModulationParams_t ModulationParams;//调制参数
}SX126x_t;

/*!
 * Hardware IO IRQ callback function definition  中断回调
 */
typedef void ( DioIrqHandler )( void );

/*!
 * SX126x definitions
 */

/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   ( double )32000000  //晶振频率
#define FREQ_DIV                                    ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                                   ( double )( XTAL_FREQ / FREQ_DIV )

#define RX_BUFFER_SIZE                              256  //接收缓冲区

/*!
 * \brief The radio callbacks structure
 * Holds function pointers to be called on radio interrupts
 */
typedef struct
{
    void ( *txDone )( void );                       //!< Pointer to a function run on successful transmission 发送完成
    void ( *rxDone )( void );                       //!< Pointer to a function run on successful reception接收完成
    void ( *rxPreambleDetect )( void );             //!< Pointer to a function run on successful Preamble detection前导码检测完成
    void ( *rxSyncWordDone )( void );               //!< Pointer to a function run on successful SyncWord reception同步字接收完成
    void ( *rxHeaderDone )( bool isOk );            //!< Pointer to a function run on successful Header reception头部接收完成
    void ( *txTimeout )( void );                    //!< Pointer to a function run on transmission timeout发送超时
    void ( *rxTimeout )( void );                    //!< Pointer to a function run on reception timeout接收超时
    void ( *rxError )( IrqErrorCode_t errCode );    //!< Pointer to a function run on reception error接收错误
    void ( *cadDone )( bool cadFlag );              //!< Pointer to a function run on channel activity detected cad检测完成
}SX126xCallbacks_t;  //定义的接口

/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */
 
/*!
 * \brief Initializes the radio driver
 */
void SX126xInit( DioIrqHandler dioIrq );//初始化射频驱动

/*!
 * \brief Gets the current Operation Mode of the Radio
 *
 * \retval      RadioOperatingModes_t last operating mode
 */
RadioOperatingModes_t SX126xGetOperatingMode( void );//获取操作模式

/*!
 * \brief Sets/Updates the current Radio OperationMode variable.
 *
 * \remark WARNING: This function is only required to reflect the current radio
 *                  operating mode when processing interrupts.
 *
 * \param [in] mode           New operating mode
 */
void SX126xSetOperatingMode( RadioOperatingModes_t mode );//设置操作模式

/*!
 * \brief Wakeup the radio if it is in Sleep mode and check that Busy is low
 */
void SX126xCheckDeviceReady( void );//检测设备是否准备好

/*!
 * \brief Saves the payload to be send in the radio buffer
 *
 * \param [in]  payload       A pointer to the payload
 * \param [in]  size          The size of the payload
 */
void SX126xSetPayload( uint8_t *payload, uint8_t size );//设置负载，保存负载数据发送给radio缓存

/*!
 * \brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * \param [out] payload       A pointer to a buffer into which the payload will be copied
 * \param [out] size          A pointer to the size of the payload received
 * \param [in]  maxSize       The maximal size allowed to copy into the buffer
 */
uint8_t SX126xGetPayload( uint8_t *payload, uint8_t *size, uint8_t maxSize );//获取负载，接收

/*!
 * \brief Sends a payload
 *
 * \param [in]  payload       A pointer to the payload to send
 * \param [in]  size          The size of the payload to send
 * \param [in]  timeout       The timeout for Tx operation
 */
void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout );//超时发送负载

/*!
 * \brief Sets the Sync Word given by index used in GFSK
 *
 * \param [in]  syncWord      SyncWord bytes ( 8 bytes )
 *
 * \retval      status        [0: OK, 1: NOK]
 */
uint8_t SX126xSetSyncWord( uint8_t *syncWord );//设置同步字

/*!
 * \brief Sets the Initial value for the LFSR used for the CRC calculation
 *
 * \param [in]  seed          Initial LFSR value ( 2 bytes )
 *
 */
void SX126xSetCrcSeed( uint16_t seed );//crc计算方法

/*!
 * \brief Sets the seed used for the CRC calculation
 *
 * \param [in]  seed          The seed value
 *
 */
void SX126xSetCrcPolynomial( uint16_t polynomial );//crc多项式

/*!
 * \brief Sets the Initial value of the LFSR used for the whitening in GFSK protocols
 *
 * \param [in]  seed          Initial LFSR value
 */
void SX126xSetWhiteningSeed( uint16_t seed );//gfsk 监听种子

/*!
 * \brief Gets a 32 bits random value generated by the radio
 *
 * \remark The radio must be in reception mode before executing this function
 *
 * \retval randomValue    32 bits random value
 */
uint32_t SX126xGetRandom( void );//获取32位随机数

/*!
 * \brief Sets the radio in sleep mode
 *
 * \param [in]  sleepConfig   The sleep configuration describing data
 *                            retention and RTC wake-up
 */
void SX126xSetSleep( SleepParams_t sleepConfig );//设置睡眠

/*!
 * \brief Sets the radio in configuration mode
 *
 * \param [in]  mode          The standby mode to put the radio into
 */
void SX126xSetStandby( RadioStandbyModes_t mode );//设置待机模式

/*!
 * \brief Sets the radio in FS mode
 */
void SX126xSetFs( void );//设置频率合成

/*!
 * \brief Sets the radio in transmission mode
 *
 * \param [in]  timeout       Structure describing the transmission timeout value
 */
void SX126xSetTx( uint32_t timeout );//设置发送模式

/*!
 * \brief Sets the radio in reception mode
 *
 * \param [in]  timeout       Structure describing the reception timeout value
 */
void SX126xSetRx( uint32_t timeout );//设置接收模式

/*!
 * \brief Sets the radio in reception mode with Boosted LNA gain
 *
 * \param [in]  timeout       Structure describing the reception timeout value
 */
void SX126xSetRxBoosted( uint32_t timeout );//设置接收模式LNA增益

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void SX126xSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );//设置占空比接收

/*!
 * \brief Sets the radio in CAD mode
 */
void SX126xSetCad( void );//设置CAD

/*!
 * \brief Sets the radio in continuous wave transmission mode
 */
void SX126xSetTxContinuousWave( void );//一直发送连续波

/*!
 * \brief Sets the radio in continuous preamble transmission mode
 */
void SX126xSetTxInfinitePreamble( void );//设置连续的前序发送模式

/*!
 * \brief Decide which interrupt will stop the internal radio rx timer.
 *
 * \param [in]  enable          [0: Timer stop after header/syncword detection
 *                               1: Timer stop after preamble detection]
 */
void SX126xSetStopRxTimerOnPreambleDetect( bool enable );//设置哪一个中断将停止接收定时器，0表示头部或者同步字检测后停止，1表示前导码检测结束后停止

/*!
 * \brief Set the number of symbol the radio will wait to validate a reception
 *
 * \param [in]  SymbNum          number of LoRa symbols
 */
void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum );//设置检测等待符号数

/*!
 * \brief Sets the power regulators operating mode
 *
 * \param [in]  mode          [0: LDO, 1:DC_DC]
 */
void SX126xSetRegulatorMode( RadioRegulatorMode_t mode );//设置电源模式

/*!
 * \brief Calibrates the given radio block
 *
 * \param [in]  calibParam    The description of blocks to be calibrated
 */
void SX126xCalibrate( CalibrationParams_t calibParam );//校准无线模块

/*!
 * \brief Calibrates the Image rejection depending of the frequency
 *
 * \param [in]  freq    The operating frequency
 */
void SX126xCalibrateImage( uint32_t freq );//根据频率校准镜像抑制

/*!
 * \brief Activate the extention of the timeout when long preamble is used
 *
 * \param [in]  enable      The radio will extend the timeout to cope with long preamble
 */
void SX126xSetLongPreamble( uint8_t enable );//在使用长序言时激活超时扩展，不懂

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  paDutyCycle     Duty Cycle for the PA   //占空比
 * \param [in]  hpMax          0 for sx1261, 7 for sx1262
 * \param [in]  deviceSel       1 for sx1261, 0 for sx1262
 * \param [in]  paLut           0 for 14dBm LUT, 1 for 22dBm LUT
 */
void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut );//pa配置

/*!
 * \brief Defines into which mode the chip goes after a TX / RX done
 *
 * \param [in]  fallbackMode    The mode in which the radio goes
 */
void SX126xSetRxTxFallbackMode( uint8_t fallbackMode );//芯片默认模式

/*!
 * \brief Write data to the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  buffer        The data to be written in radio's memory
 * \param [in]  size          The number of bytes to write in radio's memory
 */
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size );//写寄存器

/*!
 * \brief Read data from the radio memory
 *
 * \param [in]  address       The address of the first byte to read from the radio
 * \param [out] buffer        The buffer that holds data read from radio
 * \param [in]  size          The number of bytes to read from radio's memory
 */
void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size );//读寄存器

/*!
 * \brief Write data to the buffer holding the payload in the radio
 *
 * \param [in]  offset        The offset to start writing the payload
 * \param [in]  buffer        The data to be written (the payload)
 * \param [in]  size          The number of byte to be written
 */
void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );//写缓冲器

/*!
 * \brief Read data from the buffer holding the payload in the radio
 *
 * \param [in]  offset        The offset to start reading the payload
 * \param [out] buffer        A pointer to a buffer holding the data from the radio
 * \param [in]  size          The number of byte to be read
 */
void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );//读缓冲器

/*!
 * \brief   Sets the IRQ mask and DIO masks
 *
 * \param [in]  irqMask       General IRQ mask
 * \param [in]  dio1Mask      DIO1 mask
 * \param [in]  dio2Mask      DIO2 mask
 * \param [in]  dio3Mask      DIO3 mask
 */
void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );//设置中断参数

/*!
 * \brief Returns the current IRQ status
 *
 * \retval      irqStatus     IRQ status
 */
uint16_t SX126xGetIrqStatus( void );//获取中断状态

/*!
 * \brief Indicates if DIO2 is used to control an RF Switch
 *
 * \param [in] enable     true of false
 */
void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable );//设置DIO2是否用于控制射频开关

/*!
 * \brief Indicates if the Radio main clock is supplied from a tcxo
 *
 * \param [in] tcxoVoltage     voltage used to control the TCXO
 * \param [in] timeout         time given to the TCXO to go to 32MHz
 */
void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout );//设置DIO3是否作为温补晶振控制

/*!
 * \brief Sets the RF frequency
 *
 * \param [in]  frequency     RF frequency [Hz]
 */
void SX126xSetRfFrequency( uint32_t frequency );//设置无线频率

/*!
 * \brief Sets the radio for the given protocol
 *
 * \param [in]  packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA]
 *
 * \remark This method has to be called before SetRfFrequency,
 *         SetModulationParams and SetPacketParams
 */
void SX126xSetPacketType( RadioPacketTypes_t packetType );//设置包类型

/*!
 * \brief Gets the current radio protocol
 *
 * \retval      packetType    [PACKET_TYPE_GFSK, PACKET_TYPE_LORA]
 */
RadioPacketTypes_t SX126xGetPacketType( void );//获取包类型

/*!
 * \brief Sets the transmission parameters
 *
 * \param [in]  power         RF output power [-18..13] dBm
 * \param [in]  rampTime      Transmission ramp up time
 */
void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime );//设置发送参数

/*!
 * \brief Set the modulation parameters
 *
 * \param [in]  modParams     A structure describing the modulation parameters
 */
void SX126xSetModulationParams( ModulationParams_t *modParams );//设置调制参数

/*!
 * \brief Sets the packet parameters
 *
 * \param [in]  packetParams  A structure describing the packet parameters
 */
void SX126xSetPacketParams( PacketParams_t *packetParams );//设置包参数

/*!
 * \brief Sets the Channel Activity Detection (CAD) parameters 设置CAD参数
 *
 * \param [in]  cadSymbolNum   字符数The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param [in]  cadDetPeak     Limit for detection of SNR peak used in the CAD信噪比峰值
 * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
 * \param [in]  cadExitMode    Operation to be done at the end of CAD action
 *                             [LORA_CAD_ONLY, LORA_CAD_RX, LORA_CAD_LBT]  CAD退出模式
 * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity cad超时时间
 */
void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout );

/*!
 * \brief Sets the data buffer base address for transmission and reception
 *
 * \param [in]  txBaseAddress Transmission base address
 * \param [in]  rxBaseAddress Reception base address
 */
void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress );//设置缓存地址基地址

/*!
 * \brief Gets the current radio status
 *
 * \retval      status        Radio status
 */
RadioStatus_t SX126xGetStatus( void );//获取状态

/*!
 * \brief Returns the instantaneous RSSI value for the last packet received
 *
 * \retval      rssiInst      Instantaneous RSSI
 */
int8_t SX126xGetRssiInst( void );//返回最后一个包的信号强度

/*!
 * \brief Gets the last received packet buffer status
 *
 * \param [out] payloadLength Last received packet payload length
 * \param [out] rxStartBuffer Last received packet buffer address pointer
 */
void SX126xGetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBuffer );//获取缓冲区状态

/*!
 * \brief Gets the last received packet payload length
 *
 * \param [out] pktStatus     A structure of packet status
 */
void SX126xGetPacketStatus( PacketStatus_t *pktStatus );//获取包状态

/*!
 * \brief Returns the possible system errors
 *
 * \retval sysErrors Value representing the possible sys failures
 */
RadioError_t SX126xGetDeviceErrors( void );//获取设备错误

/*!
 * \brief Clear all the errors in the device
 */
void SX126xClearDeviceErrors( void );//清除设备错误

/*!
 * \brief Clears the IRQs
 *
 * \param [in]  irq           IRQ(s) to be cleared
 */
void SX126xClearIrqStatus( uint16_t irq );//清除中断状态

#endif // __SX126x_H__

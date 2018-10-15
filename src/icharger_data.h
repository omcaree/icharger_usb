#ifndef ICHARGER_DATA_H
#define ICHARGER_DATA_H

#include <inttypes.h>

#define MAX_CELLS           16
#define LIST_MEM_MAX        64
#define MODEL_MAX           2

#define VALUE_ORDER_KEY	0x55aa

enum ProgramType {
	RUNOP_CHARGE,
	RUNOP_STORAGE,
	RUNOP_DISCHARGE,
	RUNOP_CYCLE,
	RUNOP_BALANCE
};

enum OrderAction
{
	ORDER_STOP=0,	
	ORDER_RUN,
	ORDER_MODIFY,	
	ORDER_WRITE_SYS,
	ORDER_WRITE_MEM_HEAD,
	ORDER_WRITE_MEM,
	ORDER_TRANS_LOG_ON,
	ORDER_TRANS_LOG_OFF,
	ORDER_MSGBOX_YES, 
	ORDER_MSGBOX_NO, 
};

enum ModbusRequestError
{
    MB_EOK = 0x00,                      /*!< no error. */
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B,
    MB_ENOREG = 0x80,  			        /*!< illegal register address. */
    MB_EILLFUNCTION,                    /*!< illegal function code. */
    MB_EIO,                     		/*!< I/O error. */
    MB_ERETURN,                         /*!< protocol stack in illegal state. */
    MB_ELEN,                            /*!< pack len larg error. */
    MB_ETIMEDOUT                		/*!< timeout error occurred. */
};

#define REG_HOLDING_CTRL_START 0x8000
#define REG_HOLDING_CTRL_NREGS 7

enum RegistrySelectionOp
{
	REG_SEL_OP = REG_HOLDING_CTRL_START,
	REG_SEL_MEM,	
	REG_SEL_CHANNEL,		
	REG_ORDER_KEY,
	REG_ORDER,	
	REG_CURRENT,
	REG_VOLT
};

typedef uint32_t   u32;
typedef int32_t    s32;
typedef uint16_t   u16;
typedef int16_t    s16;
typedef uint8_t    u8;
typedef int8_t     s8;

#pragma pack(1)
union register16 {
    union {
        u16 value;
        s16 svalue;
    };
    
    struct {
        u8 high;
        u8 low;
    } __attribute__ ((packed));
} __attribute__ ((packed));

union register32 {	
    union {
        u32 value;	
        s32 svalue;
    };
    
    struct {
        register16 high;
        register16 low;
    } __attribute__ ((packed));
} __attribute__ ((packed));	

// available at base address: 0x0000
struct device_only {
    register16   device_id;
    s8           device_sn[12];
    register16   sw_version;
    register16   hw_version;
    register16   system_length;
    register16   memory_length;
    register16   ch1_status;
    register16   ch2_status;
} __attribute__ ((packed));

// available at 0x0100 and 0x0200 (ch1 and ch2)
struct channel_status {
    register32 timestamp;
    register32 output_power;
    register16 output_current;
    register16 input_voltage;
    register16 output_voltage;
    register32 output_capacity;
    
    register16 temp_internal;
    register16 temp_external;
    
    u16 cell_voltage[MAX_CELLS];
    u8 balance_status[MAX_CELLS];
    u16 cell_resistance[MAX_CELLS];
    u16 total_resistance;
    
    u16 line_internal_resistance;
    u16 cycle_count;	
    u16 control_status;
    u16 run_status;
    u16 run_error;
    u16 dialog_box_id;
} __attribute__ ((packed));

// system storage at 0x8400
struct system_storage {
    u16 temp_unit;                  // P1_1, 0 celcius, 1: fahrenheit
    u16 temp_cut_off;               // P1_2, 60.0 - 75.0 default 75.0
    u16 temp_fans_on;               // P1_4, 30.0 - 50.0 default 40.0
    u16 temp_power_reduce;          // P1_3, 5.0 - 20.0 default 10.0
    u16 reserved_1;
    u16 fans_off_delay;             // P1_5
    u16 lcd_contrast;               // P2_1
    u16 backlight_value;            // P2_2
    u16 reserved_2;
    u16 beep_type[4];               // long, short, continuous P3_3
    u16 beep_enabled[4];            // same shit?  P3_1
    u16 beep_volume[4];         	// P3_2
    u16 reserved_3;
    u16 calibration;                // P4_1 - whatever this means?
    u16 reserved_4;
    u16 input_source;               // 0:dc, 1:bat P5_1
    u16 dc_input_low_volt;          // DC input low voltage protection P6_1
    u16 dc_input_over_volt;         // 
    u16 dc_input_current_limit; 	// DC input current max P6_2
    u16 batt_input_low_volt;        // BATT input low volt protection P7_1
    u16 batt_input_over_volt;       // 
    s16 batt_input_current_limit;	// input current max limit P7_2
    u16 regenerative_enable;        // P7_3
    u16 regenerative_volt_limit;	// P7_4
    s16 regenerative_current_limit;	// P7_5
    u16 charger_power[MODEL_MAX];	// P8_1 and P8_3
    u16 discharge_power[MODEL_MAX]; // P8_2 and P8_4
    u16 power_priority;
    u16 logging_sample_interval;	// P9_1
    u16 logging_save_to_sdcard;     // 0:no output, 1: output to SD log P9_2
    
    u16 servo_type;                 // P10_1
    u16 servo_user_center;          // servo pulse center P10_2
    u16 servo_user_rate;            // servo frame refresh rate P10_3
    u16 servo_user_op_angle;        // 45 deg. pulse width P10_4
    
    u16 modbus_model;               // P11_1 - presume USB or serial?
    u16 modbus_serial_addr;         // serial comms address P11_4
    u16 modbus_serial_baud_rate;	// serial comms baud rate P11_2
    u16 modbus_serial_parity_bits;	// serial comms parity P11_3
    
    u16 reserved_end[8];
};

struct memory_header {
    u16 Count;                  //0—LIST_MEM_MAX
    u8 Index[LIST_MEM_MAX]; 	//0xff-- empty 0xfe--hidden 0-LIST_MEM_MAX 
};

struct control_register {
    u16 operation;
    u16 select_memory;  // values 0-63
    u16 select_channel; // 0 or 1
    u16 order_lock;     // 0x55aa unlocks
    u16 order;          // see also enum ORDER
    u16 limit_current;
    u16 limit_voltage;
};

#define MEM_HEAD_DEFAULT {7,{0,1,2,3,4,5,6}}

#define REG_MEM_START 0x8c00
#define MEM_NAME_LEN 37
struct memory {
	u16 UseFlag; //Use flag 0xffff—EMPTY 0x55aa—USED 0x0000— FIXED
	s8 Name[MEM_NAME_LEN+1]; // Program name (M1_1)
	u32 Capacity; // Nominal capacity (M1_4)
	u8 AutoSave; // The program runs automatically saved(M2_2)
	u8 LiBalEndMode; //Li -battery balance end current mode(M8_1)
	u8 Dump1[7]; // Reservation
	u16 OpEnable; //Enable module or not, represents with bit0-15 respectively
	//Charge(bit0) ,Storage(bit2) ,Discharge(bit3) ,Cycle(bit4) ,OnlyBalance(bit5)
	// (M6_5) (M10_5) (M11_4) (M12_4) (M15_1)
	u8 ChannelMode; //=0: channel asynchronous mode = 1: channel synchronous mode (M2_1)
	u8 SaveToSD; //=0:do not output log to SD =1:output log to SD (M2_5)
	u16 LogInterval; //sampling interval 0.1S as an unit (M2_4)
	u16 RunCounter; //run counter (M2_3)
	u8 Type; //Bat. Type: LiPo,LiLo,LiFe,NiMH,Nicd,Pb (M1_2)
	u8 LiCell; //Li-battery number of cells (M1_3)
	u8 NiCell; //Ni-battery number of cells (M1_3)
	u8 PbCell; //Pb-battery number of cells (M1_3)
	u8 LiModeC; //Charge mode (M6_2)
	u8 LiModeD; //Discharge mode(M14_1)
	u8 NiModeC; //Charge mode: Normal,REFLEX (M19_2)
	u8 NiModeD; //Discharge mode: Reservation
	u8 PbModeC; //Charge mode: (M16_2)
	u8 PbModeD; //Discharge mode: Reservation
	u8 BalSpeed; //Balance speed: 0--slow 1--normal 2--fast (M6_2)
	u8 BalStartMode; //Balance start mode (M7_1)
	u16 BalStartVolt; //Balance start voltage Reservation
	u8 BalDiff; // Balanced stop accuracy (mV) (M7_2)
	u8 BalOverPoint; //Balance over point(M7_4)
	u8 BalSetPoint; //The minimum voltage difference and set point when balance charge terminates
	//for example: 4.2Vcharges LiPo,BalSetPoint=5, then stops at 4.195V
	//(M7_3)
	u8 BalDelay; //Balance delay end time(M7_5)
	u8 KeepChargeEnable; //keep charging(M9_4)
	u16 LiPoChgCellVolt; //LiPo cell charge voltage (M6_4)
	u16 LiLoChgCellVolt; //LiLo cell charge voltage (M6_4)
	u16 LiFeChgCellVolt; //LiFe cell charge voltage (M6_4)
	u16 LiPoStoCellVolt; //LiPo cell storage voltage(M11_1)
	u16 LiLoStoCellVolt; //LiLo cell storage voltage(M11_1)
	u16 LiFeStoCellVolt; //LiFe cell storage voltage(M11_1)
	u16 LiPoDchgCellVolt; //LiPo cell discharge end voltage(M10_2)
	u16 LiLoDchgCellVolt; //LiLo cell discharge end voltage(M10_2)
	u16 LiFeDchgCellVolt; //LiFe cell discharge end voltage(M10_2)
	u16 ChargeCurrent; // Set charge current(M6_1)
	u16 DischargeCurrent; //Set discharge current(M10_1)
	u16 EndCharge; //Charge end current(M6_3)
	u16 EndDischarge; //Discharge end current(M10_3)
	u16 RegDchgMode; // Discharge mode(M10_4)
	u16 NiPeak; // Ni-battery sensitive voltage(M18_1)
	u16 NiPeakDelay; // deltaV check delay(M18_2)
	u16 NiTrickleEnable; //Enable trickle charge(M18_3)
	u16 NiTrickleCurrent; //Trickle charge current(M18_4)
	u16 NiTrickleTime; // Trickle charge(M18_5)
	u16 NiZeroEnable; // Ni charging 0 voltage allowed (M18_6)
	u16 NiDischargeVolt; //Ni discharge voltage (M20_2)
	u16 PbChgCellVolt; //Pb cell charge voltage (M16_4)
	u16 PbDchgCellVolt; //Pb cell discharge voltage (M17_2)
	u16 PbFloatEnable; //Pb cell float enable Reservation
	u16 PbFloatCellVolt; //Pb cell float voltage Reservation
	u16 RestoreVolt; //Low voltage restore voltage (M3_1) (M9_1)
	u16 RestoreTime; // Low voltage restore time (M3_2) (M9_2)
	u16 RestoreCurent; // Low voltage restore current (M3_3) (M9_3)
	u16 CycleCount; //Cycle count(M12_2)
	u16 CycleDelay; //Cycle interval(M12_3)
	u8 CycleMode; //Cycle mode(M12_1)
	u16 SafetyTimeC; //Safety time (M4_3)
	u16 SafetyCapC; //Safety capacity%(M4_2)
	u16 SafetyTempC; //Safety temperature(M4_1)
	u16 SafetyTimeD; //Safety time(M5_3)
	u16 SafetyCapD; //Safety capacity%(M5_2)
	u16 SafetyTempD; // Safety temperature(M5_1)
	u8 RegChMode; //Channel regenerative mode(M13_1)
	u16 RegChVolt; // Channel regenerative limited voltage(M13_2)
	u16 RegChCurrent; // Channel regenerative limited current(M13_3)
	u8 FastSto; //Li-battery fast storage (M11_3)
	u16 StoCompensation; //Storage compensation voltage(M11_2)
	u16 NiZnChgCellVolt; // NiZn cell charge voltage (M6_4)
	u16 NiZnDchgCellVolt; // NiZn cell discharge end voltage(M10_2)
	u8 NiZnCell; // NiZn-battery number of cells(M1_3)
	u8 Dump; // Reservation
};

enum MemoryRegister {
	RegUseFlag = REG_MEM_START +  0 ,
	RegName = REG_MEM_START +  2 ,
	RegCapacity = REG_MEM_START +  40 ,
	RegAutoSave = REG_MEM_START +  44 ,
	RegLiBalEndMode = REG_MEM_START +  45 ,
	RegDump1 = REG_MEM_START +  46 ,
	RegOpEnable = REG_MEM_START +  53 ,
	RegChannelMode = REG_MEM_START +  55 ,
	RegSaveToSD = REG_MEM_START +  56 ,
	RegLogInterval = REG_MEM_START +  57 ,
	RegRunCounter = REG_MEM_START +  59 ,
	RegType = REG_MEM_START +  61 ,
	RegLiCell = REG_MEM_START +  62 ,
	RegNiCell = REG_MEM_START +  63 ,
	RegPbCell = REG_MEM_START +  64 ,
	RegLiModeC = REG_MEM_START +  65 ,
	RegLiModeD = REG_MEM_START +  66 ,
	RegNiModeC = REG_MEM_START +  67 ,
	RegNiModeD = REG_MEM_START +  68 ,
	RegPbModeC = REG_MEM_START +  69 ,
	RegPbModeD = REG_MEM_START +  70 ,
	RegBalSpeed = REG_MEM_START +  71 ,
	RegBalStartMode = REG_MEM_START +  72 ,
	RegBalStartVolt = REG_MEM_START +  73 ,
	RegBalDiff = REG_MEM_START +  75 ,
	RegBalOverPoint = REG_MEM_START +  76 ,
	RegBalSetPoint = REG_MEM_START +  77 ,
	RegBalDelay = REG_MEM_START +  78 ,
	RegKeepChargeEnable = REG_MEM_START +  79 ,
	RegLiPoChgCellVolt = REG_MEM_START +  80 ,
	RegLiLoChgCellVolt = REG_MEM_START +  82 ,
	RegLiFeChgCellVolt = REG_MEM_START +  84 ,
	RegLiPoStoCellVolt = REG_MEM_START +  86 ,
	RegLiLoStoCellVolt = REG_MEM_START +  88 ,
	RegLiFeStoCellVolt = REG_MEM_START +  90 ,
	RegLiPoDchgCellVolt = REG_MEM_START +  92 ,
	RegLiLoDchgCellVolt = REG_MEM_START +  94 ,
	RegLiFeDchgCellVolt = REG_MEM_START +  96 ,
	RegChargeCurrent = REG_MEM_START +  98 ,
	RegDischargeCurrent = REG_MEM_START +  100 ,
	RegEndCharge = REG_MEM_START +  102 ,
	RegEndDischarge = REG_MEM_START +  104 ,
	RegRegDchgMode = REG_MEM_START +  106 ,
	RegNiPeak = REG_MEM_START +  108 ,
	RegNiPeakDelay = REG_MEM_START +  110 ,
	RegNiTrickleEnable = REG_MEM_START +  112 ,
	RegNiTrickleCurrent = REG_MEM_START +  114 ,
	RegNiTrickleTime = REG_MEM_START +  116 ,
	RegNiZeroEnable = REG_MEM_START +  118 ,
	RegNiDischargeVolt = REG_MEM_START +  120 ,
	RegPbChgCellVolt = REG_MEM_START +  122 ,
	RegPbDchgCellVolt = REG_MEM_START +  124 ,
	RegPbFloatEnable = REG_MEM_START +  126 ,
	RegPbFloatCellVolt = REG_MEM_START +  128 ,
	RegRestoreVolt = REG_MEM_START +  130 ,
	RegRestoreTime = REG_MEM_START +  132 ,
	RegRestoreCurent = REG_MEM_START +  134 ,
	RegCycleCount = REG_MEM_START +  136 ,
	RegCycleDelay = REG_MEM_START +  138 ,
	RegCycleMode = REG_MEM_START +  140 ,
	RegSafetyTimeC = REG_MEM_START +  141 ,
	RegSafetyCapC = REG_MEM_START +  143 ,
	RegSafetyTempC = REG_MEM_START +  145 ,
	RegSafetyTimeD = REG_MEM_START +  147 ,
	RegSafetyCapD = REG_MEM_START +  149 ,
	RegSafetyTempD = REG_MEM_START +  151 ,
	RegRegChMode = REG_MEM_START +  153 ,
	RegRegChVolt = REG_MEM_START +  154 ,
	RegRegChCurrent = REG_MEM_START +  156 ,
	RegFastSto = REG_MEM_START +  158 ,
	RegStoCompensation = REG_MEM_START +  159 ,
	RegNiZnChgCellVolt = REG_MEM_START +  161 ,
	RegNiZnDchgCellVolt = REG_MEM_START +  163 ,
	RegNiZnCell = REG_MEM_START +  165 ,
	RegDump = REG_MEM_START +  166
};

struct read_data_registers {
    register16 starting_address;
    register16 quantity_to_read;
    
    read_data_registers(u16 base_addr, u16 num_registers) {
        starting_address.high = base_addr >> 8;
        starting_address.low = (u8)(base_addr & 0x00ff);

        quantity_to_read.high = num_registers >> 8;
        quantity_to_read.low = (u8)(num_registers & 0x00ff);
    }
} __attribute__ ((packed));

#pragma pack()

enum Channel {
	CHANNEL_1,
	CHANNEL_2
};

#endif // ICHARGER_DATA_H


/*
 * bms.h
 *
 *  Created on: Oct 26, 2023
 *      Author: Bae
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

// Byte 1	Alarm
struct _status_data{
	uint32_t alarm_OverVoltage:1;
	uint32_t alarm_UnderVoltage:1;
	uint32_t alarm_HighTemp:1;
	uint32_t alarm_LowTemp:1;
	uint32_t alarm_OverCurrentCharge:1;
	uint32_t alarm_OverCurrentDischarge:1;
	uint32_t alarm_HighSoc:1;
	uint32_t alarm_LowSoc:1;

	// Byte 2	State
	uint32_t State_Standby:1;
	uint32_t State_Charging:1;
	uint32_t State_Discharging:1;
	uint32_t State_reserved:5;

	// Byte 3	Warning
	uint32_t Warning_OverVoltage:1;
	uint32_t Warning_UnderVoltage:1;
	uint32_t Warning_OverCurrentCharge:1;
	uint32_t Warning_OverCurrentDischarge:1;
	uint32_t Warning_HighTemp:1;
	uint32_t Warning_LowTemp:1;
	uint32_t Warning_HighSoc:1;
	uint32_t Warning_LowSoc:1;

	// Byte 4	Error
	uint32_t Error_reserved:8;
};
#if 0
typedef struct _SOC{
	float fVoltage;
	float fCurrunt;
}SOC;
#endif
union _status{
	uint32_t data;
	struct _status_data bit;
};
typedef struct _BMS{
	union _status Status;
	//struct _SOC Soc;
	double Soc;
	int16_t Temperature[2];	// 0 - P9, 1 - P10
	int16_t MaxTemperature[2];
	int16_t MinTemperature[2];
	int16_t CoulumbCount;
	uint16_t Voltage_avg;	// ADBMS6817 100uV/bit
	double	Current;		// A
	uint8_t EnableBalancing;
}_Bms;
_Bms Bms;


enum BROADCAST_BUS_SELELCT {
	UART1_RS232 = 0,
	UART3_RS485,
	CAN
};

struct _2byte{
	uint8_t low;
	uint8_t high;
};

struct _4byte{
	uint8_t byte[4];
};

union _hword_u{
	uint16_t data;
	struct _2byte byte;
};

union _hword{
	int16_t data;
	struct _2byte byte;
};

union _word{
	uint32_t data;
	struct _4byte byte;
};

struct _Boradcast_data
{
	union _hword_u	AvgVoltage;	// 100uV
	union _hword	BatCurrent;// signed
	union _hword_u	Soc;					// %
	union _hword_u	CellVoltage[8];	// 100uV
	union _hword	MaxTemperature[2];
	union _hword	MinTemperature[2];
	union _hword	Temperature[2];
	union _hword CoulombCount;
	union _word status;
}broadcast_data;

void BMS_Cell_Balancing(void);

void Broadcast_data(void);

#endif /* INC_BMS_H_ */

/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
******************************************************************************
* @file:    serialPrintResult.h
* @brief:   Print IO terminal functions
* @version: $Revision$
* @date:    $Date$
* Developed by: ADIBMS Software team, Bangalore, India
*****************************************************************************/
/*! @addtogroup RESULT_PRINT
*  @{
*
*/

/*! @addtogroup RESULT PRINT I/O
*  @{
*
*/

#ifndef __RESULTPRINT_H
#define __RESULTPRINT_H

#include "adbms_main.h"

void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printVoltages(uint8_t tIC, cell_asic *IC, TYPE type);
void printMemPattern(uint8_t tIC, cell_asic *IC, TYPE type);
void printCalculatedMemPattern(uint8_t tIC, cell_asic *IC, TYPE type, MEM_PG pg);
void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
void printStatusMemPattern(uint8_t tIC, cell_asic *IC);
void printCalculatedStatusMemPattern(uint8_t tIC, cell_asic *IC, MEM_PG pg);
void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type);
void printDiagnosticTestResult(uint8_t tIC, cell_asic *IC, DIAGNOSTIC_TYPE type);
void diagnosticTestResultPrint(uint8_t result);
void printWritePwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type);
void printReadPwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type);
void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type);
void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type);
void printCellOpenWireTestResult(uint8_t tIC, cell_asic *IC);
void printAuxOpenWireTestResult(uint8_t tIC, cell_asic *IC);
float getVoltage(int data);
float getCdVoltage(int data);
void printPollAdcConvTime(int count);
void printMenu(void);

#endif

/** @}*/
/** @}*/
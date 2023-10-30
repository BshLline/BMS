/*
 * EKF.h
 *
 *  Created on: 2023. 7. 18.
 *      Author: wooyo
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

// External Variables //
extern double ed_EKF_SOC, ed_EKF_V1, ed_EKF_Vt;
extern unsigned int eui_EKF_SOC_UART;

// Function Statements //
void EKF_Algorithm_Main(void);
void EKF_Algorithm_Init(void);
void SocCalculation(void);

double EKF_OCV_DIV_5Poly(double SOC);
double EKF_OCV_High_5Poly(double SOC);
double EKF_OCV_Mid_5Poly(double SOC);
double EKF_OCV_Low_5Poly(double SOC);

double EKF_OCV_Dot_DIV_5Poly(double SOC);
double EKF_OCV_Dot_High_5Poly(double SOC);
double EKF_OCV_Dot_Mid_5Poly(double SOC);
double EKF_OCV_Dot_Low_5Poly(double SOC);

#endif /* INC_EKF_H_ */

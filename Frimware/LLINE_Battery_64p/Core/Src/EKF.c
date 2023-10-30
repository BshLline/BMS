/* Extended KALMAN filter for SOC estimation */

//#include "EKF.h"
//#include "math.h"
//#include <stdint.h>
//#include "adBms_Application.h"
//#include "SocAlgorithm.h"

#include "main.h"

/* EKF Setting */
// Algorithm sampling time //
#define EKF_dT 			0.1
#define EKF_SOC_Offset	0.0		// Initial SOC error for test. This should be zero during operation.

// EKF Model parameters //
#define EKF_Cn 			144000.0	// 4 Ah x 10 Parallel x 3600s
#define EKF_R0 			0.00137		// R0: 0.0137 / 10 parallel
#define EKF_R1 			0.00107		// R1: 0.0107 / 10 parallel
#define EKF_C1			18916.0		// C1: 1891.6 x 10 parallel
// EKF Design parameters //
#define EKF_Qz			0.00001		// Initial Q value of SOC
#define EKF_QV1			0.000001	// Initial Q value of V1
#define EKF_R			400.0		// Initial R value of measurement
#define EKF_P_Init_z	5.0		// Initial Offset Compensation Factor of SOC
#define EKF_P_Init_V1	0.1			// Initial Offset Compensation Factor of V1

// Open Circuit Voltage Dividing //
// Low side: 0.1% ~ 29.9% / Middle side: 30.0% ~ 79.9% / High side: 80.0% ~ 99.0 //
#define High_SOC		0.8			// High Side SOC 100% ~ 80%
#define Mid_SOC			0.3			// Middle Side SOC 31% ~ 80%

// Upper and Lower boundary of SOC //
#define Max_SOC 0.99			// 99 %
#define Min_SOC 0.01			// 01 %

// Algorithm Initialization Flag //
unsigned char guc_EKF_Init_Flag = 0;	// 0: First call, 1: Initialization Complete

// EKF Variables //
double gd_EKF_A_Matrix[2][2], gd_EKF_C_Matrix[2], gd_EKF_K_Matrix[2], gd_EKF_Y;
double gd_EKF_Q_Matrix[2][2], gd_EKF_R = 0;
// Prediction & Innovation //
double gd_EKF_State_Matrix_Pre[2], gd_EKF_P_Matrix_Pre[2][2];
double gd_EKF_State_Matrix[2], gd_EKF_P_Matrix[2][2];

// External Variables //
double ed_EKF_SOC = 0, ed_EKF_V1 = 0, ed_EKF_Vt = 0;

// For Communication //
unsigned int eui_EKF_SOC_UART = 0;

// OCV Table (rounded to 5 decimal places and multiplied by 10000)
uint16_t OcvTable[22] = {
	25000,   //0%
    30915, //5%
    32399, //10%
    33291, //15%
    34089, //20%
    34866, //25%
    35342, //30%
    35842, //35%
    36260, //40%
    36673, //45%
    37128, //50%
    37634, //55%
    38166, //60%
    38615, //65%
    39039, //70%
    39461, //75%
    40016, //80%
    40565, //85%
    40853, //90%
    41087, //95%
	42000  //100%

};
uint16_t SOC_Init = 0;

void EKF_Algorithm_Main(void)
{
	unsigned char i, j, k;
	double d_Buff_Matrix_A[2][2], d_Buff_Matrix_B[2][2];
	double d_Buff_a;
	double d_EKF_Current_M, d_EKF_Voltage_M;

	// Buffer Clear //
	for(i=0;i<2;i++)
	{
		for(j=0;j<2;j++)
		{
			d_Buff_Matrix_A[i][j] = 0;
			d_Buff_Matrix_B[i][j] = 0;
		}
	}

	d_Buff_a = 0;

	if(guc_EKF_Init_Flag == 0)
	{
		EKF_Algorithm_Init();
		guc_EKF_Init_Flag = 1;
	}
	else if(guc_EKF_Init_Flag == 1)
	{
		//d_EKF_Current_M = g_Battery_Current1;		// put current
		//d_EKF_Current_M = (double)Bms.Current;		// put current
		//d_EKF_Current_M = (double)0.1;		// put current
		d_EKF_Current_M = (double)0.028;		// put current

		//d_EKF_Voltage_M = g_ModuleVoltageInfoArray[0].AverageVoltage/1000;		// put cell voltage
		d_EKF_Voltage_M = (double)(Bms.Voltage_avg/10000.);		// put cell voltage

		gd_EKF_State_Matrix_Pre[0] = gd_EKF_State_Matrix[0] - (d_EKF_Current_M/EKF_Cn)*EKF_dT;
		// A. Prediction of States //
		gd_EKF_State_Matrix_Pre[1] = gd_EKF_State_Matrix[1]*exp(-EKF_dT/(EKF_R1*EKF_C1))
				                     + (1 - exp(-EKF_dT/(EKF_R1*EKF_C1)))*EKF_R1*d_EKF_Current_M;

		// P_Pre = A*P*A' + Q; //
		// 1. Buff_A = A*P //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				for(k=0;k<2;k++)
				{
					d_Buff_Matrix_A[i][j] += gd_EKF_A_Matrix[i][k]*gd_EKF_P_Matrix[k][j];
				}
			}
		}
		// 2. Buff_B = Buff_A*A' //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				for(k=0;k<2;k++)
				{
					d_Buff_Matrix_B[i][j] += d_Buff_Matrix_A[i][k]*gd_EKF_A_Matrix[j][k];
				}
			}
		}
		// 3. P_Pre = Buff_B + Q //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				gd_EKF_P_Matrix_Pre[i][j] = d_Buff_Matrix_B[i][j] + gd_EKF_Q_Matrix[i][j];
			}
		}
		// 4. Buffer Clear //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				d_Buff_Matrix_A[i][j] = 0;
				d_Buff_Matrix_B[i][j] = 0;
			}
		}

		// B. Innovation of States //
		gd_EKF_C_Matrix[0] = EKF_OCV_Dot_DIV_5Poly(gd_EKF_State_Matrix_Pre[0]);
		gd_EKF_C_Matrix[1] = 1;

		// KALMAN Gain Update //
		// K = P*C'/(C*P*C' + R) //
		// 1. Buff_a = C*P*C' + R //
		d_Buff_a = gd_EKF_C_Matrix[0]*(gd_EKF_P_Matrix_Pre[0][0]*gd_EKF_C_Matrix[0] + gd_EKF_P_Matrix_Pre[0][1]*gd_EKF_C_Matrix[1])
				   + gd_EKF_C_Matrix[1]*(gd_EKF_P_Matrix_Pre[1][0]*gd_EKF_C_Matrix[0] + gd_EKF_P_Matrix_Pre[1][1]*gd_EKF_C_Matrix[1])
				   + gd_EKF_R;
		// 2. K = P*C'/Buff_a //
		gd_EKF_K_Matrix[0] = (gd_EKF_P_Matrix_Pre[0][0]*gd_EKF_C_Matrix[0] + gd_EKF_P_Matrix_Pre[0][1]*gd_EKF_C_Matrix[1])/d_Buff_a;
		gd_EKF_K_Matrix[1] = (gd_EKF_P_Matrix_Pre[1][0]*gd_EKF_C_Matrix[0] + gd_EKF_P_Matrix_Pre[1][1]*gd_EKF_C_Matrix[1])/d_Buff_a;
		// 3. Buffer Clear //
		d_Buff_a = 0;

		// Model Output //
		gd_EKF_Y = EKF_OCV_DIV_5Poly(gd_EKF_State_Matrix_Pre[0]) - gd_EKF_State_Matrix_Pre[1] - EKF_R0*d_EKF_Current_M;

		// State Update //
		gd_EKF_State_Matrix[0] = gd_EKF_State_Matrix_Pre[0] + gd_EKF_K_Matrix[0]*(d_EKF_Voltage_M - gd_EKF_Y);
		gd_EKF_State_Matrix[1] = gd_EKF_State_Matrix_Pre[1] + gd_EKF_K_Matrix[1]*(d_EKF_Voltage_M - gd_EKF_Y);

		// Covariance Matrix P Update //
		// P = (I - K*C)*P_Pre //
		// 1. Buff_A = I - K*C //
		d_Buff_Matrix_A[0][0] = 1 - gd_EKF_K_Matrix[0]*gd_EKF_C_Matrix[0];		d_Buff_Matrix_A[0][1] = -gd_EKF_K_Matrix[0]*gd_EKF_C_Matrix[1];
		d_Buff_Matrix_A[1][0] = -gd_EKF_K_Matrix[1]*gd_EKF_C_Matrix[0];			d_Buff_Matrix_A[1][1] = 1 - gd_EKF_K_Matrix[1]*gd_EKF_C_Matrix[1];
		// 2. Buff_B = Buff_A*P_Pre //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				for(k=0;k<2;k++)
				{
					d_Buff_Matrix_B[i][j] += d_Buff_Matrix_A[i][k]*gd_EKF_P_Matrix_Pre[k][j];
				}
			}
		}
		// 3. P = Buff_B
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				gd_EKF_P_Matrix[i][j] = d_Buff_Matrix_B[i][j];
			}
		}
		// 4. Buffer Clear //
		for(i=0;i<2;i++)
		{
			for(j=0;j<2;j++)
			{
				d_Buff_Matrix_A[i][j] = 0;
				d_Buff_Matrix_B[i][j] = 0;
			}
		}

		// Matrix P should be Symmetric Matrix //
		gd_EKF_P_Matrix[0][1] = (gd_EKF_P_Matrix[0][1] + gd_EKF_P_Matrix[1][0])/2;
		gd_EKF_P_Matrix[1][0] = gd_EKF_P_Matrix[0][1];

		// SOC Boundary //
		if(gd_EKF_State_Matrix[0] > Max_SOC)
		{
			gd_EKF_State_Matrix[0] = Max_SOC;
		}
		else if(gd_EKF_State_Matrix[0] < Min_SOC)
		{
			gd_EKF_State_Matrix[0] = Min_SOC;
		}
		else
		{
			// Empty //
		}

		// Estimation Results //
		ed_EKF_SOC = gd_EKF_State_Matrix[0];	// Estimated SOC
		ed_EKF_V1 = gd_EKF_State_Matrix[1];		// Estimated V1
		ed_EKF_Vt = gd_EKF_Y;					// Estimated Vt

		// Communication Data //
		eui_EKF_SOC_UART = ed_EKF_SOC*10000;

	}
}

void EKF_Algorithm_Init(void)
{
	// Initial SOC //
	SocCalculation();

	// State Initialization //
	gd_EKF_State_Matrix[0] = SOC_Init/100. + EKF_SOC_Offset;		// put initial SOC value
	gd_EKF_State_Matrix[1] = 0;		// Initial V1 value

	// Model Initialization //
	gd_EKF_A_Matrix[0][0] = 1;		gd_EKF_A_Matrix[0][1] = 0;
	gd_EKF_A_Matrix[1][0] = 0;		gd_EKF_A_Matrix[1][1] = exp(-EKF_dT/(EKF_R1*EKF_C1));

	// Design Parameter Initialization //
	gd_EKF_Q_Matrix[0][0] = EKF_Qz;			gd_EKF_Q_Matrix[0][1] = 0;
	gd_EKF_Q_Matrix[1][0] = 0;				gd_EKF_Q_Matrix[1][1] = EKF_QV1;
	gd_EKF_R = EKF_R;

	gd_EKF_P_Matrix[0][0] = EKF_P_Init_z;
	gd_EKF_P_Matrix[1][1] = EKF_P_Init_V1;
}

double EKF_OCV_DIV_5Poly(double SOC)
{
	// If the SOC is in low range //
	if(SOC < Mid_SOC)
	{
		return EKF_OCV_Low_5Poly(SOC);

	}
	// Else if the SOC is in middle range //
	else if(SOC < High_SOC)
	{
		return EKF_OCV_Mid_5Poly(SOC);
	}
	// Else if the SOC is in high range //
	else
	{
		return EKF_OCV_High_5Poly(SOC);
	}
}

double EKF_OCV_High_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.895, std = 0.05528;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {4.084, 0.02152, -0.008372, 0.0101, 0.00236, -0.0007013};
	// Result //
	double SOC_Norm = 0, OCV_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<6;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_High = p1*SOC^5 + p2*SOC^4 + p3*SOC^3 + p4*SOC^2 + p5*SOC + p6 //
		OCV_High = OCV_High + Param[i]*Pow_SOC;
	}

	return OCV_High;
}

double EKF_OCV_Mid_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.5495, std = 0.1445;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {3.764, 0.1493, 0.0003391, -0.01939, 0.0002086, 0.005026};
	// Result //
	double SOC_Norm = 0, OCV_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<6;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_High = p1*SOC^5 + p2*SOC^4 + p3*SOC^3 + p4*SOC^2 + p5*SOC + p6 //
		OCV_High = OCV_High + Param[i]*Pow_SOC;
	}

	return OCV_High;
}

double EKF_OCV_Low_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.15, std = 0.08646;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {3.326, 0.1473, 0.006766, 0.002178, -0.02616, 0.01023};
	// Result //
	double SOC_Norm = 0, OCV_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<6;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_High = p1*SOC^5 + p2*SOC^4 + p3*SOC^3 + p4*SOC^2 + p5*SOC + p6 //
		OCV_High = OCV_High + Param[i]*Pow_SOC;
	}

	return OCV_High;
}

double EKF_OCV_Dot_DIV_5Poly(double SOC)
{
	// If the SOC is in low range //
	if(SOC < Mid_SOC)
	{
		return EKF_OCV_Dot_Low_5Poly(SOC);

	}
	// Else if the SOC is in middle range //
	else if(SOC < High_SOC)
	{
		return EKF_OCV_Dot_Mid_5Poly(SOC);
	}
	// Else if the SOC is in high range //
	else
	{
		return EKF_OCV_Dot_High_5Poly(SOC);
	}
}

double EKF_OCV_Dot_High_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.895, std = 0.05528;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {4.084, 0.02152, -0.008372, 0.0101, 0.00236, -0.0007013};
	// Result //
	double SOC_Norm = 0, OCV_Dot_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<5;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_Dot_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_Dot_High = 5*p1*SOC^4 + 4*p2*SOC^3 + 3*p3*SOC^2 + 2*p4*SOC + p5 //
		OCV_Dot_High = OCV_Dot_High + (i+1)*Param[i+1]*Pow_SOC;
	}

	return OCV_Dot_High;
}

double EKF_OCV_Dot_Mid_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.5495, std = 0.1445;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {3.764, 0.1493, 0.0003391, -0.01939, 0.0002086, 0.005026};
	// Result //
	double SOC_Norm = 0, OCV_Dot_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<5;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_Dot_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_Dot_High = 5*p1*SOC^4 + 4*p2*SOC^3 + 3*p3*SOC^2 + 2*p4*SOC + p5 //
		OCV_Dot_High = OCV_Dot_High + (i+1)*Param[i+1]*Pow_SOC;
	}

	return OCV_Dot_High;
}

double EKF_OCV_Dot_Low_5Poly(double SOC)
{
	unsigned char i = 0;
	// Normalization parameters //
	double mean = 0.15, std = 0.08646;
	// coefficients of polynomials p6 ~ p1 //
	double Param[6] = {3.326, 0.1473, 0.006766, 0.002178, -0.02616, 0.01023};
	// Result //
	double SOC_Norm = 0, OCV_Dot_High = 0, Pow_SOC = 1;

	// Normalization //
	SOC_Norm = (SOC - mean)/std;

	for(i=0;i<5;i++)
	{
		if(i == 0)
		{
			Pow_SOC = 1;
			OCV_Dot_High = 0;
		}
		else
		{
			Pow_SOC = Pow_SOC*SOC_Norm;
		}

		// OCV_Dot_High = 5*p1*SOC^4 + 4*p2*SOC^3 + 3*p3*SOC^2 + 2*p4*SOC + p5 //
		OCV_Dot_High = OCV_Dot_High + (i+1)*Param[i+1]*Pow_SOC;
	}

	return OCV_Dot_High;
}


void SocCalculation(void) {
    // Local Variables //
    uint8_t i;
    double Soc_Value = 0;
    double Buffer_Ocv = 0;
    double Ocv_Value = 0;

    // Convert analog input values to SOC values for g_Soc[0]
    //Ocv_Value = (uint16_t)(g_ModuleVoltageInfoArray[0].AverageVoltage * 10); // Get Average Voltage Value
    Ocv_Value = (uint16_t)Adbms6817.voltage_avg; // Get Average Voltage Value

    for(i=0;i<21;i++)
    {
        if(Ocv_Value < OcvTable[i])
        {
            if(i==0)
            {
                Buffer_Ocv = OcvTable[0];
                Soc_Value = 0;
                //Soc_Value = ((OcvTable[0] - Ocv_Value) * 5) / (Buffer_Ocv - OcvTable[1]);
            }
            else if(i < 20)
            {
                Buffer_Ocv = (OcvTable[i] - OcvTable[i-1]);
//                Soc_Value = (i-1)*5 + ((OcvTable[i] - Ocv_Value) * 5) / Buffer_Ocv;
                Soc_Value = (i-1)*5+(5/Buffer_Ocv*(-OcvTable[i-1]+Ocv_Value));
            }
            else
            {
            	Buffer_Ocv = OcvTable[0];
            	Soc_Value = 0;
            }
            break;
        }
    }

    SOC_Init = Soc_Value;
}

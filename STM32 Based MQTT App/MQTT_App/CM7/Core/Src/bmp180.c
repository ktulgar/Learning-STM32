/*
 * bmp180.c
 *
 *  Created on: Dec 9, 2024
 *      Author: PC
 */

#include "bmp180.h"

short AC1;
short AC2;
short AC3;
unsigned short AC4;
unsigned short AC5;
unsigned short AC6;
short B1;
short B2;
short MB;
short MC;
short MD;

void BMP180_GetCoefficients()
{
	uint8_t data[22] = {0};
	HAL_I2C_Mem_Read(&hi2c1, 0xEE, 0xAA, 1, data, 22, 100);
	AC1 = data[0] << 8 | data[1];
	AC2 = data[2] << 8 | data[3];
	AC3 = data[4] << 8 | data[5];
	AC4 = data[6] << 8 | data[7];
	AC5 = data[8] << 8 | data[9];
	AC6 = data[10] << 8 | data[11];
	B1  = data[12] << 8 | data[13];
	B2  = data[14] << 8 | data[15];
	MB  = data[16] << 8 | data[17];
	MC  = data[18] << 8 | data[19];
	MD  = data[20] << 8 | data[21];
}

void BMP180_ReadData(BMP180_Data *bmp180)
{

	 long UT;
	 long T;
	 long X1;
	 long X2;
	 long B5;
     long p;

    uint8_t data[5] = {0};

    data[0] = 0x2e;
	HAL_I2C_Mem_Write(&hi2c1, 0xEE, 0xF4, 1, data, 1, 100);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, 0xEE, 0xf6, 1, data, 2, 100);
	UT = data[0] << 8 | data[1];

	data[0] = 0x34;
	HAL_I2C_Mem_Write(&hi2c1, 0xEE, 0xF4, 1, data, 1, 100);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, 0xEE, 0xf6, 1, data, 3, 100);
	long UP = ((data[0] << 16) |  (data[1] << 8) | (data[2])) >> 8;



	X1 = (UT - AC6)*AC5/32768;
	X2 = MC * 2048 / (X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8)/16;
	bmp180->temperature = T*0.1;


    long B6 =  B5 - 4000;
    X1 = (B2 * (B6 * B6/4096))/2048;
    X2 = AC2*B6/2048;
    long X3 = X1 + X2;
    long B3 = (((AC1*4+X3) << 0) + 2)/4;
    X1 = AC3 * B6 / 8192;
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;
    X3 = ((X1 + X2) + 2) / 4;
    unsigned long B4 = AC4 * (unsigned long)(X3 + 32768) / 32768;
    unsigned long  B7 = ((unsigned long) UP - B3) * (50000 >> 0);
    if(B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 / B4) * 2;
    X1 = (p/256) * (p/256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + (X1 + X2 + 3791)/16;
    bmp180->pressure = p;


    bmp180->altitude = 44330*(1-(pow(((float)p/(float)101325), 0.19029495718)));
}

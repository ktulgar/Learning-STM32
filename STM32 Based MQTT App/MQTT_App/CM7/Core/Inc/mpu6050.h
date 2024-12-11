/*
 * mpu6050.h
 *
 *  Created on: Dec 9, 2024
 *      Author: PC
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c4;

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


typedef struct
{
	float Acc_X;
	float Acc_Y;
	float Acc_Z;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
}MPU6050_Data;


void MPU6050_Init(void);
void MPU6050_ReadData(MPU6050_Data *mpu6050);

#endif /* INC_MPU6050_H_ */

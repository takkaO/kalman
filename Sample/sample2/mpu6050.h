/*
 * mpu6050.h
 *
 *  Created on: 2019/10/02
 *      Author: takkaO
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "i2c.h"

#define MPU6050_SLAVE_ADDRESS_AD_L   0b11010000
#define MPU6050_SLAVE_ADDRESS_AD_H   0b11010010

#define MPU6050_DEFAULT_SAMPLE_NUM   100

enum MPU6050_ACCEL {
	RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G
};

enum MPU6050_GYRO {
	RANGE_250DPS, RANGE_500DPS, RANGE_1000DPS, RANGE_2000DPS
};

typedef union {
	double buf[8];
	struct {
		double x_accel;
		double y_accel;
		double z_accel;
		double temperature;
		double x_gyro;
		double y_gyro;
		double z_gyro;
		double time;
	} VALUE;
} MPU6050_DATA;


typedef struct {
	double a;
	double b;
}MPU6050_LINEAR;

typedef union {
	MPU6050_LINEAR buf[7];
	struct {
		MPU6050_LINEAR x_accel;
		MPU6050_LINEAR y_accel;
		MPU6050_LINEAR z_accel;
		MPU6050_LINEAR temperature;
		MPU6050_LINEAR x_gyro;
		MPU6050_LINEAR y_gyro;
		MPU6050_LINEAR z_gyro;
	} VALUE;
}MPU6050_DRIFT_OFFSET;

void MPU6050_Set_I2C_Handler(I2C_HandleTypeDef *_hi2c);
uint8_t MPU6050_WHO_AM_I(uint16_t slave_addr);
HAL_StatusTypeDef MPU6050_Calibration_Offset(uint16_t slave_addr, MPU6050_DATA *offset, uint16_t sample_num);
HAL_StatusTypeDef MPU6050_Calibration_Drift(uint16_t slave_addr, MPU6050_DRIFT_OFFSET *drift);
MPU6050_DATA MPU6050_Remove_All_Offset(MPU6050_DATA data, MPU6050_DATA offset);
MPU6050_DATA MPU6050_Remove_All_Drift(MPU6050_DATA data, MPU6050_DRIFT_OFFSET drift);
HAL_StatusTypeDef MPU6050_Request_All_Data(uint16_t slave_addr, MPU6050_DATA *data);
HAL_StatusTypeDef MPU6050_Register_Auto_Initialize(uint16_t slave_addr);
HAL_StatusTypeDef MPU6050_Change_Gyro_Range(uint16_t slave_addr, enum MPU6050_GYRO gyro_range);
HAL_StatusTypeDef MPU6050_Change_Accel_Range(uint16_t slave_addr, enum MPU6050_ACCEL accel_range);

/***************************
 * MPU6050 Register address
 ***************************/
#define A_PWR_MGMT_1    0x6B
#define A_MOT_THR       0x1F
#define A_INT_ENABLE    0x38
#define A_INT_PIN_CFG   0x37
#define A_ACCEL_CONFIG  0x1C
#define A_GYRO_CONFIG   0x1B
#define A_WHO_AM_I      0x75

#define ACCEL_XOUT_H    0x3B
/***************************
 * MPU6050 Register
 * Datasheet revision 4.0
 ***************************/
typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t CLKSEL :3;
				uint8_t TEMP_DIS :1;
				uint8_t :1;
				uint8_t CYCLE :1;
				uint8_t SLEEP :1;
				uint8_t DEVICE_RESET :1;
			} bits;
		} DATA;
	} DITAIL;
} PWR_MGMT_1;

typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t MOT_THR :8;
			} bits;
		} DATA;
	} DITAIL;
} MOT_THR;

typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t DATA_RDY_EN :1;
				uint8_t :1;
				uint8_t :1;
				uint8_t I2C_MST_INT_EN :1;
				uint8_t FIFO_OFLOW_EN :1;
				uint8_t :1;
				uint8_t MOT_EN :1;
				uint8_t :1;
			} bits;
		} DATA;
	} DITAIL;
} INT_ENABLE;

typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t :1;
				uint8_t I2C_BYPASS_EN :1;
				uint8_t FSYNC_INT_EN :1;
				uint8_t FSYNC_INT_LEVEL :1;
				uint8_t INT_RD_CLEAR :1;
				uint8_t LATCH_INT_EN :1;
				uint8_t INT_OPEN :1;
				uint8_t INT_LEVEL :1;
			} bits;
		} DATA;
	} DITAIL;
} INT_PIN_CFG;

typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t :1;
				uint8_t :1;
				uint8_t :1;
				uint8_t FS_SEL :2;
				uint8_t ZG_ST :1;
				uint8_t YG_ST :1;
				uint8_t XG_ST :1;
			} bits;
		} DATA;
	} DITAIL;
} GYRO_CONFIG;

typedef union {
	uint8_t array[2];
	struct {
		uint8_t addr;
		union {
			uint8_t byte;
			struct {
				uint8_t :1;
				uint8_t :1;
				uint8_t :1;
				uint8_t AFS_SEL :2;
				uint8_t ZA_ST :1;
				uint8_t YA_ST :1;
				uint8_t XA_ST :1;
			} bits;
		} DATA;
	} DITAIL;
} ACCEL_CONFIG;

#endif /* MPU6050_H */

/*
 * mpu6050.c
 *
 *  Created on: 2019/10/02
 *      Author: takkaO
 */

#include "mpu6050.h"

#define ACCEL_FIRST_ARRAY_INDEX 0
#define GYRO_FIRST_ARRAY_INDEX 4

typedef union {
	unsigned char raw[14];
	signed short values[7];
	struct {
		char x_accel_l :8;
		char x_accel_h :8;
		char y_accel_l :8;
		char y_accel_h :8;
		char z_accel_l :8;
		char z_accel_h :8; // 6byte
		char t_l :8;
		char t_h :8; // 8byte
		char x_gyro_l :8;
		char x_gyro_h :8;
		char y_gyro_l :8;
		char y_gyro_h :8;
		char z_gyro_l :8;
		char z_gyro_h :8; // 14byte
	} BYTE;
} MPU6050_RAW;

I2C_HandleTypeDef *mpu6050_hi2c = (void*) 0;
typedef struct {
	double accel_sensitivity;
	double gyro_sensitivity;
	PWR_MGMT_1 pwr_mgmt_1;
	INT_ENABLE int_enable;
} MPU6050_GLOBAL_VARIABLES;
MPU6050_GLOBAL_VARIABLES mpu6050_gv;

void MPU6050_Set_I2C_Handler(I2C_HandleTypeDef *_hi2c) {
	mpu6050_hi2c = _hi2c;
}

uint8_t MPU6050_WHO_AM_I(uint16_t slave_addr) {
	HAL_StatusTypeDef ret;
	uint8_t buf[1] = { A_WHO_AM_I };
	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, (uint8_t *) &buf, 1, 10);
	if (ret != HAL_OK) {
		return 0xFF;
	}
	buf[0] = 0xFF;
	ret = HAL_I2C_Master_Receive(mpu6050_hi2c, slave_addr, (uint8_t *) &buf, 1, 100);
	if (ret != HAL_OK) {
		return 0xFF;
	}
	return (buf[0] & 0b01111110);
}

HAL_StatusTypeDef MPU6050_Calibration_Offset(uint16_t slave_addr, MPU6050_DATA *offset, uint16_t sample_num) {
	HAL_StatusTypeDef ret = HAL_OK;
	MPU6050_DATA _mpu6050;
	int i, j;

	// initialize
	for (i = 0; i < 7; i++) {
		offset->buf[i] = 0.0;
	}

	for (i = 0; i < sample_num; i++) {
		ret = MPU6050_Request_All_Data(slave_addr, &_mpu6050);
		if (ret != HAL_OK) {
			return ret;
		}

		for (j = 0; j < 7; j++) {
			offset->buf[j] = (offset->buf[j] + _mpu6050.buf[j]) / 2;
		}
	}

	offset->VALUE.z_accel -= 1;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibration_Drift(uint16_t slave_addr, MPU6050_DRIFT_OFFSET *drift) {
	MPU6050_DATA data[MPU6050_DEFAULT_SAMPLE_NUM];
	int i, j;
	HAL_StatusTypeDef ret;

	for (i=0; i<MPU6050_DEFAULT_SAMPLE_NUM; i++) {
		ret = MPU6050_Request_All_Data(slave_addr, &data[i]);
		data[i].VALUE.time = HAL_GetTick() / 1000.0;
		if (ret != HAL_OK) {
			return ret;
		}
		HAL_Delay(10);
	}

	/* Calculation mean */
	MPU6050_DATA y_;
	double x_ = 0;
	for (i=0; i<MPU6050_DEFAULT_SAMPLE_NUM; i++) {
		for (j=0; j<7; j++) {
			y_.buf[j] += data[i].buf[j];
		}
		x_ += data[i].VALUE.time;
	}
	for(j=0; j<7; j++){
		y_.buf[j] /= MPU6050_DEFAULT_SAMPLE_NUM;
	}
	x_ /= MPU6050_DEFAULT_SAMPLE_NUM;


	for (j=0; j<7; j++) {
		double a1 = 0.0;
		double a2 = 0.0;
		for (i=0; i<MPU6050_DEFAULT_SAMPLE_NUM; i++) {
			a1 += (data[i].buf[j] - y_.buf[j]) * (data[i].VALUE.time - x_);
			a2 += (data[i].VALUE.time - x_) * (data[i].VALUE.time - x_);
		}
		drift->buf[j].a = a1 / a2;
		drift->buf[j].b = y_.buf[j] - (a1 / a2) * x_;
	}

	return HAL_OK;
}

MPU6050_DATA MPU6050_Remove_All_Offset(MPU6050_DATA data, MPU6050_DATA offset) {
	for (int i = 0; i < 7; i++) {
		data.buf[i] = data.buf[i] - offset.buf[i];
	}
	return data;
}

MPU6050_DATA MPU6050_Remove_All_Drift(MPU6050_DATA data, MPU6050_DRIFT_OFFSET drift) {
	for (int i = 0; i < 7; i++) {
		data.buf[i] = data.buf[i] - (drift.buf[i].a * data.VALUE.time + drift.buf[i].b);
	}
	return data;
}

HAL_StatusTypeDef MPU6050_Request_All_Data(uint16_t slave_addr, MPU6050_DATA *data) {
	HAL_StatusTypeDef ret;
	MPU6050_RAW mpu6050_data;
	uint8_t addr = ACCEL_XOUT_H;
	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, &addr, 1, 10);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_I2C_Master_Receive(mpu6050_hi2c, slave_addr, (uint8_t *) &mpu6050_data.raw, 14, 50);
	if (ret != HAL_OK) {
		return ret;
	}
	data->VALUE.time = HAL_GetTick() / 1000.0;


	for (int i = 0; i < 7; i++) {
		// swap
		mpu6050_data.values[i] = (mpu6050_data.values[i] << 8) + (mpu6050_data.values[i] >> 8);
	}

	volatile int tmp;
	// Convert unit
	for (int i = 0; i < 3; i++) {
		data->buf[i] = mpu6050_data.values[i] / mpu6050_gv.accel_sensitivity;
		tmp = data->buf[i] * 100;
		data->buf[i] = tmp / 100.0;
	}
	data->buf[3] = mpu6050_data.values[3] / 340.0 + 36.53;
	for (int i = 4; i < 7; i++) {
		data->buf[i] = mpu6050_data.values[i] / mpu6050_gv.gyro_sensitivity;
		tmp = data->buf[i] * 100;
		data->buf[i] = tmp / 100.0;
	}

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Register_Auto_Initialize(uint16_t slave_addr) {
	HAL_StatusTypeDef ret;

	/* Setup PWR_MGMT_1 */
	mpu6050_gv.pwr_mgmt_1.DITAIL.addr = A_PWR_MGMT_1;
	mpu6050_gv.pwr_mgmt_1.DITAIL.DATA.bits.DEVICE_RESET = 0; // disable reset
	mpu6050_gv.pwr_mgmt_1.DITAIL.DATA.bits.SLEEP = 0;        // disable sleep mode
	mpu6050_gv.pwr_mgmt_1.DITAIL.DATA.bits.CYCLE = 0;        // no cycle
	mpu6050_gv.pwr_mgmt_1.DITAIL.DATA.bits.TEMP_DIS = 0;     // enable temperature sensor
	mpu6050_gv.pwr_mgmt_1.DITAIL.DATA.bits.CLKSEL = 0;       // use 8MHz oscillator

	/* Setup INT_ENABLE */
	mpu6050_gv.int_enable.DITAIL.addr = A_INT_ENABLE;
	mpu6050_gv.int_enable.DITAIL.DATA.bits.MOT_EN = 0;         // disable motion detection interrupt
	mpu6050_gv.int_enable.DITAIL.DATA.bits.FIFO_OFLOW_EN = 0; // disable FIFO buffer overflow interrupt
	mpu6050_gv.int_enable.DITAIL.DATA.bits.I2C_MST_INT_EN = 0; // disable I2C Master interrupt
	mpu6050_gv.int_enable.DITAIL.DATA.bits.DATA_RDY_EN = 0;    // disable Data Ready interrupt

	/* Setup ACCEL_CONFIG */
	ACCEL_CONFIG accel_config;
	mpu6050_gv.accel_sensitivity = 16384.0;
	accel_config.DITAIL.addr = A_ACCEL_CONFIG;
	accel_config.DITAIL.DATA.byte = 0b00000000;

	/* Setup GYRO_CONFIG */
	GYRO_CONFIG gyro_config;
	mpu6050_gv.gyro_sensitivity = 131.0;
	gyro_config.DITAIL.addr = A_GYRO_CONFIG;
	gyro_config.DITAIL.DATA.byte = 0b00000000;

	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr,
			(uint8_t *) &mpu6050_gv.pwr_mgmt_1.array, 2, 100);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr,
			(uint8_t *) &mpu6050_gv.int_enable.array, 2, 100);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, (uint8_t *) &accel_config.array, 2,
			100);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, (uint8_t *) &gyro_config.array, 2, 100);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_OK;

}

HAL_StatusTypeDef MPU6050_Change_Gyro_Range(uint16_t slave_addr, enum MPU6050_GYRO gyro_range) {
	HAL_StatusTypeDef ret;
	GYRO_CONFIG gyro_config;

	gyro_config.DITAIL.addr = A_GYRO_CONFIG;
	switch (gyro_range) {
	case RANGE_250DPS:
		mpu6050_gv.gyro_sensitivity = 131.0;
		gyro_config.DITAIL.DATA.byte = 0b00000000;
		break;
	case RANGE_500DPS:
		mpu6050_gv.gyro_sensitivity = 65.5;
		gyro_config.DITAIL.DATA.byte = 0b00001000;
		break;
	case RANGE_1000DPS:
		mpu6050_gv.gyro_sensitivity = 32.8;
		gyro_config.DITAIL.DATA.byte = 0b00010000;
		break;
	case RANGE_2000DPS:
		mpu6050_gv.gyro_sensitivity = 16.4;
		gyro_config.DITAIL.DATA.byte = 0b00011000;
		break;
	default:
		gyro_config.DITAIL.DATA.byte = 0b00000000;
		mpu6050_gv.gyro_sensitivity = 131.0;
		break;
	}

	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, (uint8_t *) &gyro_config.array, 2, 100);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Change_Accel_Range(uint16_t slave_addr, enum MPU6050_ACCEL accel_range) {
	HAL_StatusTypeDef ret;
	ACCEL_CONFIG accel_config;

	accel_config.DITAIL.addr = A_ACCEL_CONFIG;
	switch (accel_range) {
	case RANGE_2G:
		mpu6050_gv.accel_sensitivity = 16384.0;
		accel_config.DITAIL.DATA.byte = 0b00000000;
		break;
	case RANGE_4G:
		mpu6050_gv.accel_sensitivity = 8192.0;
		accel_config.DITAIL.DATA.byte = 0b00001000;
		break;
	case RANGE_8G:
		mpu6050_gv.accel_sensitivity = 4096.0;
		accel_config.DITAIL.DATA.byte = 0b00010000;
		break;
	case RANGE_16G:
		mpu6050_gv.accel_sensitivity = 2048.0;
		accel_config.DITAIL.DATA.byte = 0b00011000;
		break;
	default:
		mpu6050_gv.accel_sensitivity = 16384.0;
		accel_config.DITAIL.DATA.byte = 0b00000000;
		break;
	}

	ret = HAL_I2C_Master_Transmit(mpu6050_hi2c, slave_addr, (uint8_t *) &accel_config.array, 2,
			100);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}


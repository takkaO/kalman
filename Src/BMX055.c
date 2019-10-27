/*
 * BMX055.c
 *
 *  Created on: 2019/10/21
 *      Author: csel-pc05
 */

#include "BMX055.h"

typedef struct {
	double accl;
	double gylo;
	double mgnt;
} SENSITIVITY;
SENSITIVITY Sensitivity;

ACCL_DATA accl_offset;
GYRO_DATA gyro_offset;

ACCL_DATA _accl_data;
GYRO_DATA _gyro_data;

HAL_StatusTypeDef BMX055_GetGyro(uint8_t lsb_addr, GYRO_DATA *data) {
	HAL_StatusTypeDef ret;

	ret = BMX055_GetGyroRaw(lsb_addr, data);
	if (ret != HAL_OK) {
		return ret;
	}

	data->gyro_x -= gyro_offset.gyro_x;
	data->gyro_y -= gyro_offset.gyro_y;
	data->gyro_z -= gyro_offset.gyro_z;

	return ret;
}

HAL_StatusTypeDef BMX055_GetGyroRaw(uint8_t lsb_addr, GYRO_DATA *data) {
	HAL_StatusTypeDef ret;
	uint8_t slave_addr = (BMX055_GYRO_SLAVE_ADDR << 1) | 0x01;
	uint8_t data_size = (GYRO_Z_LSB + 2) - lsb_addr;
	ACCL_DATA_REG data_raw;
	int i;

	for (i = 0; i < 6; i++) {
		data_raw.d_array_bit[i] = 0;
	}

	ret = HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &lsb_addr, 1, 100);
	if (ret != HAL_OK) {
		return ret;
	}

	slave_addr = (BMX055_GYRO_SLAVE_ADDR << 1);
	ret = HAL_I2C_Master_Receive(&hi2c1, slave_addr, &data_raw.d_array_bit[lsb_addr - 2], data_size,
			100);
	if (ret != HAL_OK) {
		return ret;
	}

	data->gyro_x = ((data_raw.BIT.x_msb * 256) + data_raw.BIT.x_lsb);
	if (data->gyro_x > 32767) {
		data->gyro_x -= 65536;
	}
	data->gyro_y = ((data_raw.BIT.y_msb * 256) + data_raw.BIT.y_lsb);
	if (data->gyro_y > 32767) {
		data->gyro_y -= 65536;
	}
	data->gyro_z = ((data_raw.BIT.z_msb * 256) + data_raw.BIT.z_lsb);
	if (data->gyro_z > 32767) {
		data->gyro_z -= 65536;
	}

	data->gyro_x = data->gyro_x * Sensitivity.gylo;
	data->gyro_y = data->gyro_y * Sensitivity.gylo;
	data->gyro_z = data->gyro_z * Sensitivity.gylo;

	data->gyro_x = LPF_RATE * _gyro_data.gyro_x + (1.0 - LPF_RATE) * data->gyro_x;
	_gyro_data.gyro_x = data->gyro_x;
	data->gyro_y = LPF_RATE * _gyro_data.gyro_y + (1.0 - LPF_RATE) * data->gyro_y;
	_gyro_data.gyro_y = data->gyro_y;
	data->gyro_z = LPF_RATE * _gyro_data.gyro_z + (1.0 - LPF_RATE) * data->gyro_z;
	_gyro_data.gyro_z = data->gyro_z;

	return ret;
}

HAL_StatusTypeDef BMX055_GetAccl(uint8_t lsb_addr, ACCL_DATA *data) {
	HAL_StatusTypeDef ret;

	ret = BMX055_GetAcclRaw(lsb_addr, data);
	if (ret != HAL_OK) {
		return ret;
	}

	data->accl_x -= accl_offset.accl_x;
	data->accl_y -= accl_offset.accl_y;
	data->accl_z -= accl_offset.accl_z;

	return ret;
}

HAL_StatusTypeDef BMX055_GetAcclRaw(uint8_t lsb_addr, ACCL_DATA *data) {
	HAL_StatusTypeDef ret;
	uint8_t slave_addr = (BMX055_ACCL_SLAVE_ADDR << 1) | 0x01;
	uint8_t data_size = (ACCL_Z_LSB + 2) - lsb_addr;
	ACCL_DATA_REG data_raw;
	int i;

	for (i = 0; i < 6; i++) {
		data_raw.d_array_bit[i] = 0;
	}

	ret = HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &lsb_addr, 1, 100);
	if (ret != HAL_OK) {
		return ret;
	}

	slave_addr = (BMX055_ACCL_SLAVE_ADDR << 1);
	ret = HAL_I2C_Master_Receive(&hi2c1, slave_addr, &data_raw.d_array_bit[lsb_addr - 2], data_size,
			100);
	if (ret != HAL_OK) {
		return ret;
	}

	data->accl_x = ((data_raw.BIT.x_msb * 256) + (data_raw.BIT.x_lsb & 0xF0)) / 16;
	if (data->accl_x > 2047) {
		data->accl_x -= 4096;
	}
	data->accl_y = ((data_raw.BIT.y_msb * 256) + (data_raw.BIT.y_lsb & 0xF0)) / 16;
	if (data->accl_y > 2047) {
		data->accl_y -= 4096;
	}
	data->accl_z = ((data_raw.BIT.z_msb * 256) + (data_raw.BIT.z_lsb & 0xF0)) / 16;
	if (data->accl_z > 2047) {
		data->accl_z -= 4096;
	}

	data->accl_x = data->accl_x * Sensitivity.accl;
	data->accl_y = data->accl_y * Sensitivity.accl;
	data->accl_z = data->accl_z * Sensitivity.accl;

	data->accl_x = LPF_RATE * _accl_data.accl_x + (1.0 - LPF_RATE) * data->accl_x;
	_accl_data.accl_x = data->accl_x;
	data->accl_y = LPF_RATE * _accl_data.accl_y + (1.0 - LPF_RATE) * data->accl_y;
	_accl_data.accl_y = data->accl_y;
	data->accl_z = LPF_RATE * _accl_data.accl_z + (1.0 - LPF_RATE) * data->accl_z;
	_accl_data.accl_z = data->accl_z;

	return ret;
}

HAL_StatusTypeDef BMX055_InitGyro(GYRO_RANGE range, GYRO_BAND_WIDTH bw, GYRO_LPM1_REG reg) {
	HAL_StatusTypeDef ret;
	uint8_t id;
	int i;

	ret = BMX055_ReadChipID(BMX055_GYRO_SLAVE_ADDR, &id);
	if (ret != HAL_OK) {
		return ret;
	}
	if (id != BMX055_GYRO_CHIP_ID) {
		return HAL_ERROR;
	}

	switch (range) {
	case RANGE_125DPS:
		Sensitivity.gylo = 1.0 / 262.4;
		break;
	case RANGE_250DPS:
		Sensitivity.gylo = 1.0 / 131.2;
		break;
	case RANGE_500DPS:
		Sensitivity.gylo = 1.0 / 65.6;
		break;
	case RANGE_1000DPS:
		Sensitivity.gylo = 1.0 / 32.8;
		break;
	case RANGE_2000DPS:
		Sensitivity.gylo = 1.0 / 16.4;
		break;
	default:
		break;
	}

	ret = BMX055_WriteReg8(BMX055_GYRO_SLAVE_ADDR, GYRO_RANGE_ADDR, (uint8_t) range);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = BMX055_WriteReg8(BMX055_GYRO_SLAVE_ADDR, GYRO_BW_ADDR, (uint8_t) bw);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = BMX055_WriteReg8(BMX055_GYRO_SLAVE_ADDR, GYRO_LPM1_ADDR, reg.data);
	/*
	GYRO_DATA _data;
	for (i = 0; i < OFFSET_SAMPLE; i++) {
		BMX055_GetGyroRaw(GYRO_X_LSB, &_data);
		HAL_Delay(1);
	}

	for (i = 0; i < OFFSET_SAMPLE; i++) {
		BMX055_GetGyroRaw(GYRO_X_LSB, &_data);
		gyro_offset.gyro_x += _data.gyro_x;
		gyro_offset.gyro_y += _data.gyro_y;
		gyro_offset.gyro_z += _data.gyro_z;
		HAL_Delay(1);
	}
	gyro_offset.gyro_x /= OFFSET_SAMPLE;
	gyro_offset.gyro_y /= OFFSET_SAMPLE;
	gyro_offset.gyro_z /= OFFSET_SAMPLE;
*/
	return ret;
}

HAL_StatusTypeDef BMX055_InitAccl(ACCL_RANGE range, ACCL_BAND_WIDTH bw, ACCL_PMU_LPW_REG reg) {
	HAL_StatusTypeDef ret;
	uint8_t id;
	int i;

	ret = BMX055_ReadChipID(BMX055_ACCL_SLAVE_ADDR, &id);
	if (ret != HAL_OK) {
		return ret;
	}
	if (id != BMX055_ACCL_CHIP_ID) {
		return HAL_ERROR;
	}

	switch (range) {
	case RANGE_2G:
		Sensitivity.accl = 1.0 / 1024;
		break;
	case RANGE_4G:
		Sensitivity.accl = 1.0 / 512;
		break;
	case RANGE_8G:
		Sensitivity.accl = 1.0 / 256;
		break;
	case RANGE_16G:
		Sensitivity.accl = 1.0 / 128;
		break;
	default:
		break;
	}

	ret = BMX055_WriteReg8(BMX055_ACCL_SLAVE_ADDR, ACCL_PMU_RANGE_ADDR, (uint8_t) range);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = BMX055_WriteReg8(BMX055_ACCL_SLAVE_ADDR, ACCL_PMU_BW_ADDR, (uint8_t) bw);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = BMX055_WriteReg8(BMX055_ACCL_SLAVE_ADDR, ACCL_PMU_LPW_ADDR, reg.data);

	/*
	ACCL_DATA _data;
	for (i = 0; i < OFFSET_SAMPLE; i++) {
		BMX055_GetGyroRaw(GYRO_X_LSB, &_data);
		HAL_Delay(1);
	}


	for (i = 0; i < OFFSET_SAMPLE; i++) {
		BMX055_GetAcclRaw(ACCL_X_LSB, &_data);
		accl_offset.accl_x += _data.accl_x;
		accl_offset.accl_y += _data.accl_y;
		accl_offset.accl_z += _data.accl_z - 1.0;
		HAL_Delay(1);
	}
	accl_offset.accl_x /= OFFSET_SAMPLE;
	accl_offset.accl_y /= OFFSET_SAMPLE;
	accl_offset.accl_z /= OFFSET_SAMPLE;
	*/

	return ret;
}

HAL_StatusTypeDef BMX055_ReadChipID(uint8_t slave_addr, uint8_t *id) {
	HAL_StatusTypeDef ret;

	switch (slave_addr) {
	case BMX055_ACCL_SLAVE_ADDR:
		ret = BMX055_ReadReg8(slave_addr, ACCL_CHIP_ID_ADDR, id);
		break;
	case BMX055_GYRO_SLAVE_ADDR:
		ret = BMX055_ReadReg8(slave_addr, GYRO_CHIP_ID_ADDR, id);
		break;
	case BMX055_MGNT_SLAVE_ADDR:
		ret = BMX055_WriteReg8(BMX055_MGNT_SLAVE_ADDR, MGNT_PWR_CTRL_ADDR, 0x01);
		if (ret != HAL_OK) {
			return ret;
		}
		ret = BMX055_ReadReg8(slave_addr, MGNT_CHIP_ID_ADDR, id);
		break;
	default:
		ret = HAL_ERROR;
	}
	return ret;
}

HAL_StatusTypeDef BMX055_WriteReg8(uint8_t slave_addr, uint8_t addr, uint8_t data) {
	HAL_StatusTypeDef ret;

	uint8_t i2cData[2] = { addr, data };
	slave_addr = (slave_addr << 1) | 0x01;

	ret = HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &i2cData[0], 2, 100);
	return ret;
}

HAL_StatusTypeDef BMX055_ReadReg8(uint8_t slave_addr, uint8_t addr, uint8_t *data) {
	HAL_StatusTypeDef ret;

	slave_addr = (slave_addr << 1);

	ret = HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &addr, 1, 100);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, slave_addr, data, 1, 100);
	return ret;
}

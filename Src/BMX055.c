/*
 * BMX055.c
 *
 *  Created on: 2019/10/21
 *      Author: csel-pc05
 */

#include "BMX055.h"

HAL_StatusTypeDef BMX055_InitGyro(GYRO_RANGE range, GYRO_BAND_WIDTH bw, GYRO_LPM1_REG reg) {
	HAL_StatusTypeDef ret;
	uint8_t id;

	ret = BMX055_ReadChipID(BMX055_GYRO_SLAVE_ADDR, &id);
	if (ret != HAL_OK) {
		return ret;
	}
	if (id != BMX055_GYRO_CHIP_ID) {
		return HAL_ERROR;
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

	return ret;
}

HAL_StatusTypeDef BMX055_InitAccl(ACCL_RANGE range, ACCL_BAND_WIDTH bw, ACCL_PMU_LPW_REG reg) {
	HAL_StatusTypeDef ret;
	uint8_t id;

	ret = BMX055_ReadChipID(BMX055_ACCL_SLAVE_ADDR, &id);
	if (ret != HAL_OK) {
		return ret;
	}
	if (id != BMX055_ACCL_CHIP_ID) {
		return HAL_ERROR;
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

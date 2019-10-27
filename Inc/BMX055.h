/*
 * BMX055.h
 *
 *  Created on: 2019/10/21
 *      Author: csel-pc05
 */

#ifndef BMX055_H_
#define BMX055_H_

#include "i2c.h"

#define BMX055_ACCL_SLAVE_ADDR   0x19
#define BMX055_GYRO_SLAVE_ADDR   0x69
#define BMX055_MGNT_SLAVE_ADDR   0x13

#define BMX055_ACCL_CHIP_ID      0xFA
#define BMX055_GYRO_CHIP_ID      0x0F
#define BMX055_MGNT_CHIP_ID      0x32

#define OFFSET_SAMPLE    200
#define LPF_RATE         0.0

typedef enum {
	RANGE_2G  = 0b00000011,
	RANGE_4G  = 0b00001010,
	RANGE_8G  = 0b00001000,
	RANGE_16G = 0b00001100
}ACCL_RANGE;

typedef enum {
	BW_7_81_Hz  = 0b00001000,
	BW_15_63_Hz = 0b00001001,
	BW_31_25_Hz = 0b00001010,
	BW_62_5_Hz  = 0b00001011,
	BW_125_Hz   = 0b00001100,
	BW_250_Hz   = 0b00001101,
	BW_500_HZ   = 0b00001110,
	BW_1000_Hz  = 0b00001111
}ACCL_BAND_WIDTH;


typedef enum {
	RANGE_2000DPS = 0b00000000,
	RANGE_1000DPS = 0b00000001,
	RANGE_500DPS  = 0b00000010,
	RANGE_250DPS  = 0b00000011,
	RANGE_125DPS = 0b00000100
}GYRO_RANGE;

typedef enum {
	BW_523_Hz = 0x0000000,
	BW_230_Hz = 0x0000001,
	BW_116_Hz = 0x0000010,
	BW_47_Hz  = 0x0000011,
	BW_23_Hz  = 0x0000100,
	BW_12_Hz  = 0x0000101,
	BW_64_Hz  = 0x0000110,
	BW_32_Hz  = 0x0000111,
}GYRO_BAND_WIDTH;

/* Accel register address */
#define ACCL_CHIP_ID_ADDR        0x00
#define ACCL_PMU_RANGE_ADDR      0x0F
#define ACCL_PMU_BW_ADDR         0x10
#define ACCL_PMU_LPW_ADDR        0x11
#define ACCL_X_LSB               0x02
#define ACCL_Y_LSB               0x04
#define ACCL_Z_LSB               0x06

/* Accel register structure */
typedef union {
	uint8_t data;
	struct {
		uint8_t :1;
		uint8_t sleep_dur    :4;
		uint8_t deep_suspend :1;
		uint8_t lowpower_en  :1;
		uint8_t suspend      :1;
	}BIT;
}ACCL_PMU_LPW_REG;

typedef struct {
	double accl_x;
	double accl_y;
	double accl_z;
}ACCL_DATA;

typedef union {
	uint8_t d_array_bit[6];
	struct {
		uint8_t x_lsb :8;
		uint8_t x_msb :8;
		uint8_t y_lsb :8;
		uint8_t y_msb :8;
		uint8_t z_lsb :8;
		uint8_t z_msb :8;
	}BIT;
}ACCL_DATA_REG;

/* Gyro register address */
#define GYRO_CHIP_ID_ADDR        0x00
#define GYRO_RANGE_ADDR          0x0F
#define GYRO_BW_ADDR             0x10
#define GYRO_LPM1_ADDR           0x11
#define GYRO_X_LSB               0x02
#define GYRO_Y_LSB               0x04
#define GYRO_Z_LSB               0x06

/* Gyro register structure */
typedef union {
	uint8_t data;
	struct {
		uint8_t :1;
		uint8_t sleep_dur    :3;
		uint8_t :1;
		uint8_t deep_suspend :1;
		uint8_t :1;
		uint8_t suspend      :1;
	}BIT;
}GYRO_LPM1_REG;

typedef struct {
	double gyro_x;
	double gyro_y;
	double gyro_z;
}GYRO_DATA;

typedef union {
	uint8_t d_array_bit[6];
	struct {
		uint8_t x_lsb :8;
		uint8_t x_msb :8;
		uint8_t y_lsb :8;
		uint8_t y_msb :8;
		uint8_t z_lsb :8;
		uint8_t z_msb :8;
	}BIT;
}GYRO_DATA_REG;

/* Magnetometer register address */
#define MGNT_CHIP_ID_ADDR        0x40
#define MGNT_PWR_CTRL_ADDR       0x4B


/* Prototypes */
HAL_StatusTypeDef BMX055_GetGyro(uint8_t lsb_addr, GYRO_DATA *data);
HAL_StatusTypeDef BMX055_GetGyroRaw(uint8_t lsb_addr, GYRO_DATA *data);
HAL_StatusTypeDef BMX055_GetAccl(uint8_t lsb_addr, ACCL_DATA *data);
HAL_StatusTypeDef BMX055_GetAcclRaw(uint8_t lsb_addr, ACCL_DATA *data);
HAL_StatusTypeDef BMX055_InitAccl(ACCL_RANGE range, ACCL_BAND_WIDTH bw, ACCL_PMU_LPW_REG reg);
HAL_StatusTypeDef BMX055_InitGyro(GYRO_RANGE range, GYRO_BAND_WIDTH bw, GYRO_LPM1_REG reg);
HAL_StatusTypeDef BMX055_ReadChipID(uint8_t slave_addr, uint8_t *id);

HAL_StatusTypeDef BMX055_WriteReg8(uint8_t slave_addr, uint8_t addr, uint8_t data);
HAL_StatusTypeDef BMX055_ReadReg8(uint8_t slave_addr, uint8_t addr, uint8_t *data);

#endif /* BMX055_H_ */

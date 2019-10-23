/*
 * mpu6050.hpp
 *
 *  Created on: 2016/10/23
 *      Author: accelerator
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "I2CMaster2.h"
#include "p24FJ64GA002.h"
#include "stdint.h"

extern unsigned long Global_Timer; 

#define A_MPU6050 0x68 //0b11010000
#define ACCEL_RANGE_2G 2
#define ACCEL_RANGE_4G 4
#define ACCEL_RANGE_8G 8
#define ACCEL_RANGE_16G 16
#define GYRO_RANGE_250 250
#define GYRO_RANGE_500 500
#define GYRO_RANGE_1000 1000
#define GYRO_RANGE_2000 2000
#define FILTER_NONE 0
#define FILTER_LPF 1
#define FILTER_HPF 2
#define FILTER_LPF_HPF 3
#define FILTER_HPF_LPF 4
#define ACCEL_FIRST_ARRAY_INDEX 0
#define GYRO_FIRST_ARRAY_INDEX 4

extern float Accel_sensitivity;
extern float Gyro_sensitivity;

//データ格納用
union AGDATA {
	unsigned char agraw[14];
	signed short agvalue[7];
	struct {
		char x_accel_h :8;
		char x_accel_l :8;
		char y_accel_h :8;
		char y_accel_l :8;
		char z_accel_h :8;
		char z_accel_l :8; //6byte
		char t_h :8;
		char t_l :8; //8byte
		char x_gyro_h :8;
		char x_gyro_l :8;
		char y_gyro_h :8;
		char y_gyro_l :8;
		char z_gyro_h :8;
		char z_gyro_l :8; //14byte
	} byte;
	struct {
		signed short x_accel :16;
		signed short y_accel :16;
		signed short z_accel :16;
		signed short temperature :16;
		signed short x_gyro :16;
		signed short y_gyro :16;
		signed short z_gyro :16;
	} value;
};

union OFFSET{
	signed short ag_offset[7];
	struct {
		signed short accel_x_offset :16;
		signed short accel_y_offset :16;
		signed short accel_z_offset :16;
		signed short temperature :16;
		signed short gyro_x_offset :16;
		signed short gyro_y_offset :16;
		signed short gyro_z_offset :16;
	}byte;
};

extern union AGDATA agdata;
extern union OFFSET offset;

//プロトタイプ宣言
void Delete_Offset(void);
signed short HPF(signed short now_data, signed short prev_data);
signed short LPF(signed short now_data, signed short prev_data);
int Calc_Offset(unsigned long sample_time, char filter, char start_num, char num);
void Calc_Accel_Offset(unsigned long sample_time, char filter);
void Calc_Gyro_Offset(unsigned long sample_time, char filter);
void swap(void);
void MPU6050_Init(int accel_range, int gyro_range);
void Request_Accel_Gyro_Data(void);

/*********************
 * MPU6050レジスタアドレス
 *********************/
#define A_PWR_MGMT_1 0x6B
#define A_MOT_THR 0x1F
#define A_INT_ENABLE 0x38
#define A_INT_PIN_CFG 0x37
#define A_ACCEL_CONFIG 0x1C
#define A_GYRO_CONFIG 0x1B

/*********************
 * MPU6050レジスタ
 *********************/

union pwr_mgmt_1 {
	uint8_t byte;
	struct {
		uint8_t CLKSEL :3;
		uint8_t TEMP_DIS :1;
		uint8_t :1;
		uint8_t CYCLE :1;
		uint8_t SLEEP :1;
		uint8_t DEVICE_RESET :1;
	} bits;
};

extern union pwr_mgmt_1 PWR_MGMT_1;

union mot_thr {
	uint8_t byte;
	struct {
		uint8_t MOT_THR :8;
	} bits;
};

extern union mot_thr MOT_THR;

union int_enable {
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
};

extern union int_enable INT_ENABLE;

union int_pin_cfg {
	uint8_t byte;
	struct {
		uint8_t :1;
		uint8_t I2C_BYPASS_EN:1;
		uint8_t FSYNC_INT_EN:1;
		uint8_t FSYNC_INT_LEVEL:1;
		uint8_t INT_RD_CLEAR :1;
		uint8_t LATCH_INT_EN:1;
		uint8_t INT_OPEN:1;
		uint8_t INT_LEVEL:1;
	} bits;
};

extern union int_pin_cfg INT_PIN_CFG;


#endif /* MPU6050_HPP_ */

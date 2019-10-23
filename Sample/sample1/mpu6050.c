#include "mpu6050.h"

union AGDATA agdata;
union OFFSET offset;

float Accel_sensitivity;
float Gyro_sensitivity;

union pwr_mgmt_1 PWR_MGMT_1;
union mot_thr MOT_THR;
union int_enable INT_ENABLE;
union int_pin_cfg INT_PIN_CFG;

/******************************************************************************************
 * @function  	:Delete_Offset
 * @explain		:取得データからオフセットを除去する関数
 * @caution		オフセットを事前に求めておくこと
 *****************************************************************************************/
void Delete_Offset(void) {
	int i;

	for (i = 0; i < 7; i++) {
		agdata.agvalue[i] -= offset.ag_offset[i];
	}
}

/******************************************************************************************
 * @function  	:Calc_Offset
 * @explain		:オフセットを求める関数
 * @parameter 	[unsigned long](sample_time)サンプリング時間(ms)
 * @parameter 	[char](filter)使用するフィルタの選択（マクロ定義済み）
 * @parameter 	[char](start_num)フィルタをかけるデータ配列の最初のインデックス
 * @parameter 	[char](num)フィルタをかけるデータの個数
 *****************************************************************************************/
int Calc_Offset(unsigned long sample_time, char filter, char start_num, char num) {
	int i;
	signed short prev_HPF[7], prev_LPF[7];
	unsigned long time_stamp;
	char end_num = start_num + num;

	//引数エラー
	if (end_num < 1 || 7 < end_num) {
		return 1;
	}

	Request_Accel_Gyro_Data();
	time_stamp = Global_Timer + sample_time;
	switch (filter) {
	case FILTER_NONE:
		while (time_stamp > Global_Timer) {
			for (i = start_num; i < end_num; i++) {
				Request_Accel_Gyro_Data();
				offset.ag_offset[i] = (agdata.agvalue[i] + offset.ag_offset[i]) / 2;
			}
		}
		break;
	case FILTER_LPF:
		for (i = start_num; i < end_num; i++) {
			prev_LPF[i] = LPF(agdata.agvalue[i], 0);
		}
		while (time_stamp > Global_Timer) {
			for (i = start_num; i < end_num; i++) {
				Request_Accel_Gyro_Data();
				//prev_LPF[i] = LPF(agdata.agvalue[i] - offset.ag_offset[i], prev_LPF[i]);
				prev_LPF[i] = LPF(agdata.agvalue[i], prev_LPF[i]);
				offset.ag_offset[i] = (prev_LPF[i] + offset.ag_offset[i]) / 2;
			}
		}
		break;
	case FILTER_HPF:
		for (i = start_num; i < end_num; i++) {
			prev_HPF[i] = HPF(agdata.agvalue[i], 0);
		}
		while (time_stamp > Global_Timer) {
			for (i = start_num; i < end_num; i++) {
				Request_Accel_Gyro_Data();
				//prev_HPF[i] = HPF(agdata.agvalue[i] - offset.ag_offset[i], prev_HPF[i]);
				prev_HPF[i] = HPF(agdata.agvalue[i + 4], prev_HPF[i]);
				offset.ag_offset[i] = (prev_HPF[i] + offset.ag_offset[i]) / 2;
			}
		}
		break;
	case FILTER_LPF_HPF:
		for (i = start_num; i < end_num; i++) {
			prev_LPF[i] = LPF(agdata.agvalue[i], 0);
			prev_HPF[i] = HPF(agdata.agvalue[i], 0);
		}
		while (time_stamp > Global_Timer) {
			for (i = start_num; i < end_num; i++) {
				Request_Accel_Gyro_Data();
				//prev_LPF[i] = LPF(agdata.agvalue[i] - offset.ag_offset[i], prev_LPF[i]);
				prev_LPF[i] = LPF(agdata.agvalue[i], prev_LPF[i]);
				prev_HPF[i] = HPF(prev_LPF[i], prev_HPF[i]);
				offset.ag_offset[i] = (prev_HPF[i] + offset.ag_offset[i]) / 2;
			}
		}
		break;
	case FILTER_HPF_LPF:
		for (i = start_num; i < end_num; i++) {
			prev_LPF[i] = LPF(agdata.agvalue[i], 0);
			prev_HPF[i] = HPF(agdata.agvalue[i], 0);
		}
		while (time_stamp > Global_Timer) {
			for (i = start_num; i < end_num; i++) {
				Request_Accel_Gyro_Data();
				//prev_HPF[i] = HPF(agdata.agvalue[i] - offset.ag_offset[i], prev_HPF[i]);
				prev_HPF[i] = HPF(agdata.agvalue[i], prev_HPF[i]);
				prev_LPF[i] = LPF(prev_HPF[i], prev_LPF[i]);
				offset.ag_offset[i] = (prev_LPF[i] + offset.ag_offset[i]) / 2;
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

/******************************************************************************************
 * @function  	:Calc_Accel_Offset
 * @explain		:3軸加速度のオフセットを求める関数
 * @parameter 	[unsigned long](sample_time)サンプリング時間(ms)
 * @parameter 	[char](filter)使用するフィルタの選択（マクロ定義済み）
 * @caution		重力を使用する場合は使用してはいけない
 *****************************************************************************************/
void Calc_Accel_Offset(unsigned long sample_time, char filter) {
	Calc_Offset(sample_time, filter, ACCEL_FIRST_ARRAY_INDEX, 3);
}

/******************************************************************************************
 * @function  	:Calc_Gyro_Offset
 * @explain		:3軸ジャイロのオフセットを求める関数
 * @parameter 	[unsigned long](sample_time)サンプリング時間(ms)
 * @parameter 	[char](filter)使用するフィルタの選択（マクロ定義済み）
 * @caution		もしかしたら1軸ごとに個別設定をしたほうがいいのかも・・・
 *****************************************************************************************/
void Calc_Gyro_Offset(unsigned long sample_time, char filter) {
	Calc_Offset(sample_time, filter, GYRO_FIRST_ARRAY_INDEX, 3);
}

/******************************************************************************************
 * @function  	:HPF
 * @explain		:ディジタルハイパスフィルタ
 * @parameter 	[signed short](now_data)今回取得した値
 * @parameter 	[signed short](prev_data)前回の値
 * @return		[signed short]今回取得した値にハイパスフィルタをかけたもの
 * @caution		現在地からLPFの結果を引くとHPFになるらしい
 *****************************************************************************************/
signed short HPF(signed short now_data, signed short prev_data) {
	now_data -= LPF(now_data, prev_data);
	return now_data;
}

/******************************************************************************************
 * @function  	:LPF
 * @explain		:ディジタルローパスフィルタ
 * @parameter 	[signed short](now_data)今回取得した値
 * @parameter 	[signed short](prev_data)前回の値
 * @return		[signed short]今回取得した値にローパスフィルタをかけたもの
 * @caution		移動平均にする場合は、前回の返り値をprev_dataに入れる
 *****************************************************************************************/
signed short LPF(signed short now_data, signed short prev_data) {
	return (signed short) (0.9 * prev_data + 0.1 * now_data);
}

/******************************************************************************************
 * @function  	:Setup_INT_PIN_CFG
 * @explain		:割り込みピンの設定
 * @caution		詳細はデータシート参照
 *****************************************************************************************/
void Setup_INT_PIN_CFG(void) {
	INT_PIN_CFG.bits.INT_LEVEL = 0;
	INT_PIN_CFG.bits.INT_OPEN = 0;
	INT_PIN_CFG.bits.LATCH_INT_EN = 0;
	INT_PIN_CFG.bits.INT_RD_CLEAR = 0;
	INT_PIN_CFG.bits.FSYNC_INT_LEVEL = 0;
	INT_PIN_CFG.bits.FSYNC_INT_EN = 0;
}

/******************************************************************************************
 * @function  	:Setup_INT_ENABLE
 * @explain		:割り込みの設定
 * @caution		詳細はデータシート参照
 *****************************************************************************************/
void Setup_INT_ENABLE(void) {
	//1有効, 0,無効
	INT_ENABLE.bits.MOT_EN = 0;
	INT_ENABLE.bits.FIFO_OFLOW_EN = 0;
	INT_ENABLE.bits.I2C_MST_INT_EN = 0;
	INT_ENABLE.bits.DATA_RDY_EN = 0;
}

/******************************************************************************************
 * @function  	:Setup_MOT_THR
 * @explain		:モーション検知閾値の設定
 * @caution		詳細はデータシート参照
 *****************************************************************************************/
void Setup_MOT_THR(void) {
	MOT_THR.bits.MOT_THR = 1;
}

/******************************************************************************************
 * @function  	:Setup_PWR_MGMT_1
 * @explain		:パワー系の設定
 * @caution		詳細はデータシート参照
 *****************************************************************************************/
void Setup_PWR_MGMT_1(void) {
	PWR_MGMT_1.bits.DEVICE_RESET = 0; //リセットしない
	PWR_MGMT_1.bits.SLEEP = 0; //スリープモード解除
	PWR_MGMT_1.bits.CYCLE = 0; //サイクルなし
	PWR_MGMT_1.bits.TEMP_DIS = 0; //温度センサ有効化
	PWR_MGMT_1.bits.CLKSEL = 0; //8MHzオシレータ使用
}

/******************************************************************************************
 * @function  	:MPU6050_Init
 * @explain		:MPU6050の初期化関数
 * @parameter 	[int](accel_range)加速度のレンジ
 * @parameter 	[int](gyro_range)ジャイロのレンジ
 * @caution		引数にはマクロ定義されたモノを使用することを推奨
 *****************************************************************************************/
void MPU6050_Init(int accel_range, int gyro_range) {
	uint8_t pwr_mgmt_1[2] = { A_PWR_MGMT_1, 0x00 };
	uint8_t mot_thr[2] = { A_MOT_THR, 0x00 };
	uint8_t int_enable[2] = { A_INT_ENABLE, 0x00 };
	uint8_t int_pin_cfg[2] = { A_INT_PIN_CFG, 0x00 };
	uint8_t accel_config[2] = { A_ACCEL_CONFIG, 0x00 };
	uint8_t gyro_config[2] = { A_GYRO_CONFIG, 0x00 };

	Setup_PWR_MGMT_1();
	Setup_MOT_THR();
	Setup_INT_PIN_CFG();
	Setup_INT_ENABLE();

	pwr_mgmt_1[1] = PWR_MGMT_1.byte;
	mot_thr[1] = MOT_THR.byte;
	int_pin_cfg[1] = INT_PIN_CFG.byte;
	int_enable[1] = INT_ENABLE.byte;

	//加速度レンジと感度定数設定
	switch (accel_range) {
	case ACCEL_RANGE_2G:
		accel_config[1] = 0b00000000;
		Accel_sensitivity = 16384.0;
		break;
	case ACCEL_RANGE_4G:
		accel_config[1] = 0b00001000;
		Accel_sensitivity = 8192.0;
		break;
	case ACCEL_RANGE_8G:
		accel_config[1] = 0b00010000;
		Accel_sensitivity = 4096.0;
		break;
	case ACCEL_RANGE_16G:
		accel_config[1] = 0b00011000;
		Accel_sensitivity = 2048.0;
		break;
	default:
		accel_config[1] = 0b00000000;
		Accel_sensitivity = 16384.0;
		break;
	}

    //ジャイロレンジと感度定数設定
	switch (gyro_range) {
	case GYRO_RANGE_250:
		gyro_config[1] = 0b00000000;
		Gyro_sensitivity = 131.0;
		break;
	case GYRO_RANGE_500:
		gyro_config[1] = 0b00001000;
		Gyro_sensitivity = 65.5;
		break;
	case GYRO_RANGE_1000:
		gyro_config[1] = 0b00010000;
		Gyro_sensitivity = 32.8;
		break;
	case GYRO_RANGE_2000:
		gyro_config[1] = 0b00011000;
		Gyro_sensitivity = 16.4;
		break;
	default:
		gyro_config[1] = 0b00000000;
		Gyro_sensitivity = 131.0;
		break;
	}

	//各種設定送信
	I2C1Send(A_MPU6050, 2, int_pin_cfg);
	I2C1Send(A_MPU6050, 2, pwr_mgmt_1);
	I2C1Send(A_MPU6050, 2, accel_config);
	I2C1Send(A_MPU6050, 2, gyro_config);
}

/******************************************************************************************
 * @function  	:swap
 * @explain		:データ交換関数
 * @caution		加速度センサの返り値HLが逆なので
 *****************************************************************************************/
void swap(void) {
	int i;
	char tmp;

	for (i = 0; i < 14; i += 2) {
		tmp = agdata.agraw[i];
		agdata.agraw[i] = agdata.agraw[i + 1];
		agdata.agraw[i + 1] = tmp;
	}
}

/******************************************************************************************
 * @function  	:Request_Accel_Gyro_Data
 * @explain		:データ取得関数
 *****************************************************************************************/
void Request_Accel_Gyro_Data(void) {
	uint8_t accel_xout_h_adress = 0x3B;
	int send, read;
	send = I2C1Send(A_MPU6050, 1, &accel_xout_h_adress);
	read = I2C1Receive(A_MPU6050, 14, agdata.agraw);
	swap();
    /*
	if((send | read) == 1){
		debug.printf("%d, %d : ", send, read);
		//I2C_test(&i2c1);
	}
    */
}


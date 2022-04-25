/*
 * MC3479_acc.c
 *
 *  Created on: 21 Apr 2022
 *      Author: Jessica Fayard
 *
 *	Pins :
 *      Supply voltage (DVDD) : 3.3V
 *      Ground (GND)
 *      Bus (SDA and SCL with pull up resistor ~4.7kΩ) : I2C
 *
 *  Initialization:
 *		ACC_MC3479_init(name_of_I2C_use);
 *
 *	Default values :
 *      SR 100Hz
 *      Range 8G
 *      No filter
 *
 *  Variable to get the raw data :
 *  	Type : ACC_XYZ_TypeDef
 *
 *  Variable to get the data in g:
 *  	Type : ACC_XYZ_g_TypeDef
 *
 *  Variable to get the offsets values :
 *  	Type : OFFSET_XYZ_TypeDef
 *
 *  Variable to get the gains values :
 *  	Type : GAINS_XYZ_TypeDef
 *
 */


//Includes
#include "MC3479_acc.h"
#include "main.h"
#include <string.h>
#include <stdio.h>


// Variables
HAL_StatusTypeDef rate_status, mode_status, filter_status, range_status, X_offset_status, Y_offset_status, Z_offset_status;
I2C_HandleTypeDef I2C;
uint8_t buf[19]={0}; // Buffer
uint8_t buf_r=0; // Read value
uint8_t range_global = MC3479_RANGE_DEFAULT; // Chosen range
int16_t X_offset,Y_offset,Z_offset;
uint16_t X_gain,Y_gain,Z_gain;


//Functions
void ACC_MC3479_init(I2C_HandleTypeDef i2c){
	I2C=i2c;

	// Configuration of the rate of the device
	ACC_MC3479_set_rate(MC3479_SAMPLE_RATE_DEFAULT);
	READ((uint16_t)MC3479_REG_SR, &buf_r);

	// Configuration of the filter and range of the device
	ACC_MC3479_set_filter(MC3479_FILTER_DEFAULT);
	ACC_MC3479_set_range(MC3479_RANGE_DEFAULT);
	READ((uint16_t)MC3479_REG_RANGE_C, &buf_r);

	// Verification of the mode of the device --> WAKE 1
	mode_status = READ((uint16_t)MC3479_REG_DEV_STAT, &buf_r);
}

void ACC_MC3479_set_mode(uint8_t mode){
	mode_status = READ((uint16_t)MC3479_REG_MODE, &buf_r);
	if (mode == MC3479_MODE_STANDBY){
		buf[0] = (buf_r & 0b11000000) | MC3479_MODE_STANDBY;
		mode_status = SEND(MC3479_REG_MODE,buf);
		if ( mode_status != HAL_OK ) {strcpy((char*)buf, "Pb mode\r\n");}
	}
	if (mode == MC3479_MODE_WAKE){
		buf[0] = (buf_r & 0b11000000) | MC3479_MODE_WAKE;
		mode_status = SEND(MC3479_REG_MODE,buf);
		if ( mode_status != HAL_OK ) {strcpy((char*)buf, "Pb mode\r\n");}
	}
	HAL_Delay(10);
}

void ACC_MC3479_set_rate(uint8_t rate_value){
	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	buf[0] = rate_value; // Data
	rate_status = SEND(MC3479_REG_SR,buf);
	if ( rate_status != HAL_OK ) {strcpy((char*)buf, "Pb rate\r\n");}
	HAL_Delay(10);

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_filter(uint8_t filter_value){
	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ((uint16_t)MC3479_REG_RANGE_C, &buf_r);
	buf[0] = (buf_r & 0b11110000) | filter_value;
	filter_status = SEND(MC3479_REG_RANGE_C,buf);
	if ( filter_status != HAL_OK ) {strcpy((char*)buf, "Pb filter\r\n");}
	HAL_Delay(10);

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_range(uint8_t range_value){
	range_global=range_value;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ((uint16_t)MC3479_REG_RANGE_C, &buf_r);
	buf[0] = (buf_r & 0b00001111) | range_value<<4;
	range_status = SEND(MC3479_REG_RANGE_C,buf);
	if ( range_status != HAL_OK ) {strcpy((char*)buf, "Pb range\r\n");}
	HAL_Delay(10);

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_X_offset(int16_t X_offset_value){
	uint8_t XH;
	uint8_t X_offset_value_LSB,X_offset_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_XOFFH,buf);
	XH=buf[0] & 0b10000000;
	X_offset_value_LSB = (uint8_t) (X_offset_value & 0b0000000011111111);
	X_offset_value_MSB = (uint8_t) ((X_offset_value & 0b0111111100000000)>>8);
	buf[0] = X_offset_value_LSB;
	X_offset_status = SEND(MC3479_REG_XOFFL,buf);
	HAL_Delay(10);
	buf[0] = XH|X_offset_value_MSB;
	X_offset_status = SEND(MC3479_REG_XOFFH,buf);
	HAL_Delay(10);
	if ( X_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb X offset\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_Y_offset(int16_t Y_offset_value){
	uint8_t YH;
	uint8_t Y_offset_value_LSB,Y_offset_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_YOFFH,buf);
	YH=buf[0] & 0b10000000;
	Y_offset_value_LSB = (uint8_t) (Y_offset_value & 0b0000000011111111);
	Y_offset_value_MSB = (uint8_t) ((Y_offset_value & 0b0111111100000000)>>8);
	buf[0] = Y_offset_value_LSB;
	Y_offset_status = SEND(MC3479_REG_YOFFL,buf);
	HAL_Delay(10);
	buf[0] = YH|Y_offset_value_MSB;
	X_offset_status = SEND(MC3479_REG_YOFFH,buf);
	HAL_Delay(10);
	if ( Y_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb Y offset\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_Z_offset(int16_t Z_offset_value){
	uint8_t ZH;
	uint8_t Z_offset_value_LSB,Z_offset_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_ZOFFH,buf);
	ZH=buf[0] & 0b10000000;
	Z_offset_value_LSB = (uint8_t) (Z_offset_value & 0b0000000011111111);
	Z_offset_value_MSB = (uint8_t) ((Z_offset_value & 0b0111111100000000)>>8);
	buf[0] = Z_offset_value_LSB;
	Z_offset_status = SEND(MC3479_REG_ZOFFL,buf);
	HAL_Delay(10);
	buf[0] = ZH|Z_offset_value_MSB;
	Z_offset_status = SEND(MC3479_REG_ZOFFH,buf);
	HAL_Delay(10);
	if ( Z_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb Z offset\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_read_offsets(OFFSET_XYZ_TypeDef *offsets){
	uint8_t XL,XH,YL,YH,ZL,ZH;
	READ(MC3479_REG_XOFFL,buf);
	XL=buf[0];
	READ(MC3479_REG_XOFFH,buf);
	XH=buf[0];
	offsets->x_offset = (int16_t)(((uint16_t)(XH & 0b01111111)<<9)|XL<<1)/2;
	HAL_Delay(10);

	READ(MC3479_REG_YOFFL,buf);
	YL=buf[0];
	READ(MC3479_REG_YOFFH,buf);
	YH=buf[0];
	offsets->y_offset = (int16_t)(((uint16_t)(YH & 0b01111111)<<9)|YL<<1)/2;
	HAL_Delay(10);

	READ(MC3479_REG_ZOFFL,buf);
	ZL=buf[0];
	READ(MC3479_REG_ZOFFH,buf);
	ZH=buf[0];
	offsets->z_offset = (int16_t)(((uint16_t)(ZH & 0b01111111)<<9)|ZL<<1)/2;
	HAL_Delay(10);
}

void ACC_MC3479_set_X_gain(uint16_t X_gain_value){
	uint8_t XH;
	uint8_t X_gain_value_LSB,X_gain_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_XOFFH,buf);
	XH=buf[0] & 0b01111111;

	X_gain_value_LSB = (uint8_t)  (X_gain_value & 0b0000000011111111);
	X_gain_value_MSB = (uint8_t) ((X_gain_value & 0b0000000100000000)>>1);
	buf[0] = X_gain_value_LSB;
	X_offset_status = SEND(MC3479_REG_XGAIN,buf);
	HAL_Delay(10);
	buf[0] = XH | X_gain_value_MSB;
	X_offset_status = SEND(MC3479_REG_XOFFH,buf);
	HAL_Delay(10);
	if ( X_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb X gain\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_Y_gain(uint16_t Y_gain_value){
	uint8_t YH;
	uint8_t Y_gain_value_LSB,Y_gain_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_YOFFH,buf);
	YH=buf[0] & 0b01111111;

	Y_gain_value_LSB = (uint8_t)  (Y_gain_value & 0b0000000011111111);
	Y_gain_value_MSB = (uint8_t) ((Y_gain_value & 0b0000000100000000)>>1);
	buf[0] = Y_gain_value_LSB;
	Y_offset_status = SEND(MC3479_REG_YGAIN,buf);
	HAL_Delay(10);
	buf[0] = YH | Y_gain_value_MSB;
	Y_offset_status = SEND(MC3479_REG_YOFFH,buf);
	HAL_Delay(10);
	if ( Y_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb Y gain\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_set_Z_gain(uint16_t Z_gain_value){
	uint8_t ZH;
	uint8_t Z_gain_value_LSB,Z_gain_value_MSB;

	ACC_MC3479_set_mode(MC3479_MODE_STANDBY);

	READ(MC3479_REG_ZOFFH,buf);
	ZH=buf[0] & 0b01111111;

	Z_gain_value_LSB = (uint8_t)  (Z_gain_value & 0b0000000011111111);
	Z_gain_value_MSB = (uint8_t) ((Z_gain_value & 0b0000000100000000)>>1);
	buf[0] = Z_gain_value_LSB;
	Z_offset_status = SEND(MC3479_REG_ZGAIN,buf);
	HAL_Delay(10);
	buf[0] = ZH | Z_gain_value_MSB;
	Z_offset_status = SEND(MC3479_REG_ZOFFH,buf);
	HAL_Delay(10);
	if ( Z_offset_status != HAL_OK ) {strcpy((char*)buf, "Pb Z gain\r\n");}

	ACC_MC3479_set_mode(MC3479_MODE_WAKE);
}

void ACC_MC3479_read_gains(GAINS_XYZ_TypeDef *gains){
	uint8_t XL,XH,YL,YH,ZL,ZH;
	READ(MC3479_REG_XGAIN,buf);
	XL=buf[0];
	READ(MC3479_REG_XOFFH,buf);
	XH=buf[0];
	gains->x_gain = (uint16_t)(((XH & 0b10000000)<<1)|XL);
	HAL_Delay(10);

	READ(MC3479_REG_YGAIN,buf);
	YL=buf[0];
	READ(MC3479_REG_YOFFH,buf);
	YH=buf[0];
	gains->y_gain = (uint16_t)(((YH & 0b10000000)<<1)|YL);
	HAL_Delay(10);

	READ(MC3479_REG_ZGAIN,buf);
	ZL=buf[0];
	READ(MC3479_REG_ZOFFH,buf);
	ZH=buf[0];
	gains->z_gain = (uint16_t)(((ZH & 0b10000000)<<1)|ZL);
	HAL_Delay(10);
}

void ACC_MC3479_ReadXYZ(ACC_XYZ_TypeDef *acc,ACC_XYZ_g_TypeDef *acc_g){
	// Value per bit (mg/LSB) {2g, 4g, 8g, 16g, 12g}
	float value_per_bit [5] = {0.061f, 0.122f, 0.244f, 0.488f, 0.366f};

	HAL_I2C_Mem_Read(&I2C, MC3479_ADDR_R, (uint16_t)MC3479_REG_XOUT_LSB, I2C_MEMADD_SIZE_8BIT, buf, (uint16_t) 6, 1000);
	HAL_Delay(10);

	//Combine the bytes
	acc->x = (int16_t)((((uint16_t)buf[1]) << 8) | (uint16_t)buf[0]);
	acc->y = (int16_t)((((uint16_t)buf[3]) << 8) | (uint16_t)buf[2]);
	acc->z = (int16_t)((((uint16_t)buf[5]) << 8) | (uint16_t)buf[4]);

	//  Transform raw data
	acc_g->x_g = (float) (((float) acc->x) * value_per_bit[range_global] / 1000.0);
	acc_g->y_g = (float) (((float) acc->y) * value_per_bit[range_global] / 1000.0);
	acc_g->z_g = (float) (((float) acc->z) * value_per_bit[range_global] / 1000.0);
}

HAL_StatusTypeDef READ(uint16_t Address, uint8_t *pData){
	return (HAL_I2C_Mem_Read(&I2C, (uint16_t)MC3479_ADDR_R, (uint16_t)Address, I2C_MEMADD_SIZE_8BIT, pData, 1, 1000));
}

HAL_StatusTypeDef SEND(uint16_t Address, uint8_t *pData){
	return (HAL_I2C_Mem_Write(&I2C, (uint16_t)MC3479_ADDR_W, (uint16_t)Address, I2C_MEMADD_SIZE_8BIT, pData, 1, 1000));
}

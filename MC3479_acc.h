/*
 * MC3479_acc.h
 *
 *  Created on: 21 Apr 2022
 *      Author: Jessica Fayard
 */

#ifndef INC_MC3479_ACC_H_
#define INC_MC3479_ACC_H_


//Includes
#include "main.h"


//Structures
	//Raw data
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} ACC_XYZ_TypeDef;

	//Data in g
typedef struct {
	float x_g;
	float y_g;
	float z_g;
} ACC_XYZ_g_TypeDef;

	//Offsets
typedef struct {
	int16_t x_offset;
	int16_t y_offset;
	int16_t z_offset;
} OFFSET_XYZ_TypeDef;

	//Gains
typedef struct {
	uint16_t x_gain;
	uint16_t y_gain;
	uint16_t z_gain;
} GAINS_XYZ_TypeDef;

	//Mode
typedef enum
{
  MC3479_MODE_STANDBY   = 0b00000000,
  MC3479_MODE_WAKE      = 0b00000001,
}   MC3479_mode_t;


	//Range
typedef enum
{
  MC3479_RANGE_2G    		= 0b0000,
  MC3479_RANGE_4G    		= 0b0001,
  MC3479_RANGE_DEFAULT_8G   = 0b0010,
  MC3479_RANGE_16G   		= 0b0011,
  MC3479_RANGE_12G   		= 0b0100,
}   MC3479_range_t;

	//Sample rate
typedef enum
{
  MC3479_SR_25Hz            = 0x10,
  MC3479_SR_50Hz            = 0x11,
  MC3479_SR_62_5Hz          = 0x12,
  MC3479_SR_DEFAULT_100Hz   = 0x13,
  MC3479_SR_125Hz           = 0x14,
  MC3479_SR_250Hz           = 0x15,
  MC3479_SR_500Hz           = 0x16,
  MC3479_SR_1000Hz  		= 0x17,
}   MC3479_sr_t;

	//Filter
typedef enum
{
  MC3479_DEFAULT_NOFILTER       = 0b0001,
  MC3479_FILTER_IDR_DIV_4_255   = 0b1001,
  MC3479_FILTER_IDR_DIV_6       = 0b1010,
  MC3479_FILTER_IDR_DIV_12   	= 0b1011,
  MC3479_FILTER_IDR_DIV_16      = 0b1101,
}   MC3479_filter_t;


//Registers addresses
#define MC3479_REG_DEV_STAT         (0x05)
#define MC3479_REG_MODE             (0x07)
#define MC3479_REG_SR               (0x08)

#define MC3479_REG_XOUT_LSB         (0x0D)
#define MC3479_REG_XOUT_MSB         (0x0E)
#define MC3479_REG_YOUT_LSB         (0x0F)
#define MC3479_REG_YOUT_MSB         (0x10)
#define MC3479_REG_ZOUT_LSB         (0x11)
#define MC3479_REG_ZOUT_MSB         (0x12)

#define MC3479_REG_RANGE_C          (0x20)

#define MC3479_REG_XOFFL            (0x21)
#define MC3479_REG_XOFFH            (0x22)
#define MC3479_REG_YOFFL            (0x23)
#define MC3479_REG_YOFFH            (0x24)
#define MC3479_REG_ZOFFL            (0x25)
#define MC3479_REG_ZOFFH            (0x26)
#define MC3479_REG_XGAIN            (0x27)
#define MC3479_REG_YGAIN            (0x28)
#define MC3479_REG_ZGAIN            (0x29)

#define MC3479_ADDR_R 				(0x98)
#define MC3479_ADDR_W				(0x99)


// Defaults values
#define MC3479_SAMPLE_RATE_DEFAULT    MC3479_SR_DEFAULT_100Hz
#define MC3479_RANGE_DEFAULT          MC3479_RANGE_DEFAULT_8G
#define MC3479_FILTER_DEFAULT         MC3479_DEFAULT_NOFILTER


// Functions prototype
HAL_StatusTypeDef SEND(uint16_t port, uint8_t *pData); 					//I2C send 1 byte function
HAL_StatusTypeDef READ(uint16_t Address, uint8_t *pData); 				//I2C read 1 byte function
void ACC_MC3479_init(I2C_HandleTypeDef i2c); 							//Sensor MC3479 initialization function
void ACC_MC3479_ReadXYZ(ACC_XYZ_TypeDef*acc,ACC_XYZ_g_TypeDef *acc_g);	//Read accelerometer function and stock data on 'acc' for raw data and 'acc_g' for data in g
void ACC_MC3479_set_mode(uint8_t mode);									//Set mode of the sensor function
void ACC_MC3479_set_rate(uint8_t rate_value);							//Set sample rate of the sensor function
void ACC_MC3479_set_filter(uint8_t filter_value);						//Set filter of the sensor function
void ACC_MC3479_set_range(uint8_t range_value);							//Set range of the sensor function
void ACC_MC3479_set_X_offset(int16_t X_offset_value); 					//X offset set function with 14-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_set_Y_offset(int16_t Y_offset_value); 					//Y offset set function with 14-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_set_Z_offset(int16_t Z_offset_value); 					//Z offset set function with 14-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_read_offsets(OFFSET_XYZ_TypeDef *offset);				//Read offsets function and stock data on 'offset'
void ACC_MC3479_set_X_gain(uint16_t X_gain_value); 						//X gain set function with 9-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_set_Y_gain(uint16_t Y_gain_value); 						//Y gain set function with 9-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_set_Z_gain(uint16_t Z_gain_value); 						//Z gain set function with 9-bits value : !!!Overwrites the value in register!!!
void ACC_MC3479_read_gains(GAINS_XYZ_TypeDef *gains);					//Read gains function and stock data on 'gains'


#endif /* INC_MC3479_ACC_H_ */

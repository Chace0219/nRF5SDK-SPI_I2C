/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File

Created By Jin zhouyun
Original Creation Date: November 16, 2016

Distributed as-is; no warranty is given.
******************************************************************************/

#include "nrf_gpio.h"
#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

/*
#include <Wire.h> // Wire library is used for I2C
#include <SPI.h>  // SPI library is used for...SPI.

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
*/

#define LSM9DS1_COMMUNICATION_TIMEOUT 1000

nrf_drv_spi_t const * b_LSMSPI;

IMUSettings settings;

// We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(), readAccel(), and readMag() first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer

int16_t temperature; // Chip temperature
float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

// x_mAddress and gAddress store the I2C address or SPI chip select pin
// for each sensor.
uint8_t _mAddress, _xgAddress;

// gRes, aRes, and mRes store the current resolution for each sensor. 
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
float gRes, aRes, mRes;

// _autoCalc keeps track of whether we're automatically subtracting off
// accelerometer and gyroscope bias calculated in calibrate().
bool _autoCalc;


float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

void LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	init(interface, xgAddr, mAddr);
}

void init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	settings.device.commInterface = interface;
	settings.device.agAddress = xgAddr;
	settings.device.mAddress = mAddr;

	settings.gyro.enabled = true;
	settings.gyro.enableX = true;
	settings.gyro.enableY = true;
	settings.gyro.enableZ = true;
	// gyro scale can be 245, 500, or 2000
	settings.gyro.scale = 245;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	settings.gyro.sampleRate = 6;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 0;
	settings.gyro.lowPowerEnable = false;
	settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	settings.gyro.HPFCutoff = 0;
	settings.gyro.flipX = false;
	settings.gyro.flipY = false;
	settings.gyro.flipZ = false;
	settings.gyro.orientation = 0;
	settings.gyro.latchInterrupt = true;

	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = -1;
	settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;

	settings.mag.enabled = true;
	// mag scale can be 4, 8, 12, or 16
	settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	settings.mag.sampleRate = 7;
	settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	settings.mag.XYPerformance = 3;
	settings.mag.ZPerformance = 3;
	settings.mag.lowPowerEnable = false;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	settings.mag.operatingMode = 0;

	settings.temp.enabled = true;
	for (int i=0; i<3; i++)
	{
		gBias[i] = 0;
		aBias[i] = 0;
		mBias[i] = 0;
		gBiasRaw[i] = 0;
		aBiasRaw[i] = 0;
		mBiasRaw[i] = 0;
	}
	_autoCalc = false;
}


uint16_t IMUbegin()
{
	//! Todo: don't use _xgAddress or _mAddress, duplicating memory
	_xgAddress = settings.device.agAddress;
	_mAddress = settings.device.mAddress;
	
	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable
	
	initSPI();	// Initialize SPI
		
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;
	
	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;
	
	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void initGyro()
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	}
	switch (settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 3);
			break;
		case 2000:
			tempRegValue |= (0x3 << 3);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);
	
	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);	
	
	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);
	
	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);
	
	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
}

void initAccel()
{
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);
	
	xgWriteByte(CTRL_REG5_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
	}
	switch (settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void calibrate(bool autoCalc) // = true)
{  
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};
	
	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
	}
	for(ii = 0; ii < samples ; ii++) 
	{	// Read the gyro data stored in the FIFO
		readGyro();
		gBiasRawTemp[0] += gx;
		gBiasRawTemp[1] += gy;
		gBiasRawTemp[2] += gz;
		readAccel();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}  
	for (ii = 0; ii < 3; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		gBias[ii] = calcGyro(gBiasRaw[ii]);
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = calcAccel(aBiasRaw[ii]);
	}
	
	enableFIFO(false);
	setFIFO(FIFO_OFF, 0x00);
	
	if (autoCalc) _autoCalc = true;
}

void calibrateMag(bool loadIn)// = true)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior
	
	for (i=0; i<128; i++)
	{
		while (!magAvailable(ALL_AXIS))
			;
		readMag();
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = mx;		
		magTemp[1] = my;
		magTemp[2] = mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = calcMag(mBiasRaw[j]);
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
	
}
void magOffset(uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
	mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

void initMag()
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);
	
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);
	
	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
}

uint8_t accelAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return (status & (1<<0));
}

uint8_t gyroAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<1)) >> 1);
}

uint8_t tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<2)) >> 2);
}

uint8_t magAvailable(lsm9ds1_axis axis)
{
	uint8_t status;
	status = mReadByte(STATUS_REG_M);
	
	return ((status & (1<<axis)) >> axis);
}

void readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
	if (_autoCalc)
	{
		ax -= aBiasRaw[X_AXIS];
		ay -= aBiasRaw[Y_AXIS];
		az -= aBiasRaw[Z_AXIS];
	}
}

int16_t readAccelAxis(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2);
	value = (temp[1] << 8) | temp[0];
	
	if (_autoCalc)
		value -= aBiasRaw[axis];
	
	return value;
}

void readMag()
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

int16_t readMagAxis(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	mReadBytes(OUT_X_L_M + (2 * axis), temp, 2);
	return (temp[1] << 8) | temp[0];
}

void readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
	temperature = ((int16_t)temp[1] << 8) | temp[0];
}

void readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
	if (_autoCalc)
	{
		gx -= gBiasRaw[X_AXIS];
		gy -= gBiasRaw[Y_AXIS];
		gz -= gBiasRaw[Z_AXIS];
	}
}

int16_t readGyroAxis(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	
	xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2);
	
	value = (temp[1] << 8) | temp[0];
	
	if (_autoCalc)
		value -= gBiasRaw[axis];
	
	return value;
}

float calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro; 
}

float calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void setGyroScale(uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			settings.gyro.scale = 245;
			break;
	}
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);
	
	calcgRes();	
}

void setAccelScale(uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;
	
	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void setMagScale(uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	
	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		settings.mag.scale = 4;
		break;
	}	
	
	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
}

void setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
}

void setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void calcgRes()
{
	gRes = ((float) settings.gyro.scale) / 32768.0;
}

void calcaRes()
{
	aRes = ((float) settings.accel.scale) / 32768.0;
}

void calcmRes()
{
	//mRes = ((float) settings.mag.scale) / 32768.0;
	switch (settings.mag.scale)
	{
	case 4:
		mRes = magSensitivity[0];
		break;
	case 8:
		mRes = magSensitivity[1];
		break;
	case 12:
		mRes = magSensitivity[2];
		break;
	case 16:
		mRes = magSensitivity[3];
		break;
	}
	
}

void configInt(interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	xgWriteByte(CTRL_REG8, temp);
}

void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	xgWriteByte(ACT_THS, temp);
	
	xgWriteByte(ACT_DUR, duration);
}

uint8_t getInactivity()
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t getAccelIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	xgWriteByte(INT_GEN_CFG_G, temp);
}

void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
}

uint8_t getGyroIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);
	
	mWriteByte(INT_CFG_M, config);
}

void configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, (uint8_t)(threshold & 0x00FF));
}

uint8_t getMagIntSrc()
{
	uint8_t intSrc = mReadByte(INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
}

void sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
}

void enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
}

void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t getFIFOSamples()
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}

void constrainScales()
{
	if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && 
		(settings.gyro.scale != 2000))
	{
		settings.gyro.scale = 245;
	}
		
	if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
		(settings.accel.scale != 8) && (settings.accel.scale != 16))
	{
		settings.accel.scale = 2;
	}
		
	if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
		(settings.mag.scale != 12) && (settings.mag.scale != 16))
	{
		settings.mag.scale = 4;
	}
}

void xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	SPIwriteByte(_xgAddress, subAddress, data);
}

void mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	SPIwriteByte(_mAddress, subAddress, data);
}

uint8_t xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	return SPIreadByte(_xgAddress, subAddress);
}

void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	SPIreadBytes(_xgAddress, subAddress, dest, count);
}

uint8_t mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	return SPIreadByte(_mAddress, subAddress);
}

void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	SPIreadBytes(_mAddress, subAddress, dest, count);
}

void initSPI()
{
	nrf_gpio_cfg_output(_xgAddress); //pinMode(_xgAddress, OUTPUT);
	nrf_gpio_pin_set(_xgAddress); // digitalWrite(_xgAddress, HIGH);
	nrf_gpio_cfg_output(_mAddress); // pinMode(_mAddress, OUTPUT);
	nrf_gpio_pin_set(_mAddress); // digitalWrite(_mAddress, HIGH);
	
	/*
	SPI.begin();
	// Maximum SPI frequency is 10MHz, could divide by 2 here:
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	// Data is read and written MSb first.
	SPI.setBitOrder(MSBFIRST);
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	SPI.setDataMode(SPI_MODE0);
	*/
}

void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data)
{
	nrf_gpio_pin_clear(csPin); // digitalWrite(csPin, LOW); // Initiate communication
	
	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	SPItransfer(subAddress & 0x3F);
	SPItransfer(data);
	nrf_gpio_pin_set(csPin); // digitalWrite(csPin, HIGH); // Close communication
}

uint8_t SPItransfer(uint8_t txData)
{
	uint8_t p_tx_data;
	uint8_t p_rx_data;
	p_tx_data = txData;
	uint32_t err_code = nrf_drv_spi_transfer(b_LSMSPI, &p_tx_data, 1, &p_rx_data, 1);
	//APP_ERROR_CHECK(err_code);
	return p_rx_data;
}

uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte. 
	// Value is returned to `temp`.
	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

void SPIreadBytes(uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{

	// To indicate a read, set bit 0 (msb) of first byte to 1
	uint8_t rAddress = 0x80 | (subAddress & 0x3F);
	// Mag SPI port is different. If we're reading multiple bytes, 
	// set bit 1 to 1. The remaining six bytes are the address to be read
	if ((csPin == _mAddress) && count > 1)
		rAddress |= 0x40;
	
	nrf_gpio_pin_clear(csPin); // digitalWrite(csPin, LOW); // Initiate communication
	SPItransfer(rAddress);	// SPI.transfer(rAddress);
	for (int i=0; i<count; i++)
	{
		dest[i] = SPItransfer(0x00); // SPI.transfer(0x00); // Read into destination array
	}
	nrf_gpio_pin_set(csPin); // digitalWrite(csPin, HIGH); // Close communication
}

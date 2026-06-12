#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "NuMicro.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_I2C_MASTER_BASE (I2C1_BASE)

#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define I2C_SLAVE_CLOCK_FREQUENCY (12000000)


#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

static const uint8_t I2C_ADDRESS1       =               (0x40);    // 1000000 (A0=GND, A1=GND)
static const uint8_t I2C_ADDRESS2       =               (0x41);    // 1000001 (A0=VCC, A1=GND)
static const uint8_t I2C_ADDRESS3       =               (0x44);    // 1000100 (A0=GND, A1=VCC)
static const uint8_t I2C_ADDRESS4       =               (0x45);    // 1000101 (A0=VCC, A1=VCC)



#define I2C_BAUDRATE (100000) /* 100K */
#define I2C_DATA_LENGTH (32)  /* MAX is 256 */

/*******************************************************************************
 * Variables
 ******************************************************************************/


// config register address
const uint8_t REG_CONFIG         	=             	(0x00);


//
const uint8_t REG_ADC_CONFIG     	=             	(0x01);


const uint8_t REG_SHUNT_CAL			=        		(0x02);

// bus voltage register
const uint8_t REG_VSHUNT       		=           	(0x04);

// power register
const uint8_t REG_VBUS             	=          		(0x05);

// current register
const uint8_t REG_DIETEMP           =         		(0x06);

// calibration register
const uint8_t REG_CURRENT       	=         		(0x07);

const uint8_t REG_POWER	       		=         		(0x08);


uint32_t _calValue;
	// The following multipliers are used to convert raw current and power
	// values to mA and mW, taking into account the current config settings
uint32_t _currentDivider_mA = 0;
float    _powerMultiplier_mW = 0;



//static i2c_master_handle_t g_m_handle;
//static status_t reVal = kStatus_Fail;
//static i2c_master_transfer_t masterXfer = {0};

//static uint8_t g_ua8DateTime[8];

uint8_t au8TxBuff[32];
uint8_t au8RxBuff[32];

volatile bool g_u8RTC_complted = true;

/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t u8NumberToBCD(uint8_t u8Num)
{
	uint8_t u8Digit_2 = u8Num / 10;
	uint8_t u8Digit_1 = u8Num % 10;
	if(u8Num > 99) return 0xff;

	return ((u8Digit_2 << 4) + u8Digit_1);
}

uint8_t u8BCDToNumber(uint8_t u8BCD)
{
	uint8_t u8Digit_2 = (u8BCD >> 4) * 10;
	uint8_t u8Digit_1 = u8BCD & 0x0f;

	return u8Digit_2 + u8Digit_1;
}

//static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *
//userData)
//{
//	/* Signal transfer success when received success status. */
//	if (status == kStatus_Success)
//	{
//		g_u8RTC_complted = true;
//	}
//}

bool bI2C_WriteBytes(uint8_t u8StartAddress,uint8_t *pu8Bytes, uint8_t u8Len)
{
	uint8_t u8Inx = 0;

//	assert(u8Len < 31);
	
	if(I2C_WriteMultiBytesOneReg(I2C1, I2C_ADDRESS1,u8StartAddress, pu8Bytes, u8Len) == u8Len) 
		return true; 
	
	return false;
}

bool bI2C_ReadBytes(uint8_t u8StartAddress, uint8_t *pu8Bytes, uint8_t u8Len)
{
	uint8_t u8GetNum;
		
	u8GetNum = I2C_ReadMultiBytesOneReg(I2C1, I2C_ADDRESS1, u8StartAddress, pu8Bytes, u8Len);
	
	
	if(u8GetNum == u8Len) 
		return true;
	
	return false;

}

void vINA237_Init()
{
	/* Open I2C module and set bus clock */
    I2C_Open(I2C1, 100000);
}


/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
bool wireWriteRegister (uint8_t reg, uint16_t value)
{
	uint8_t buffer[2];// = {(uint8_t)((value >> 8) & 0xFF), 
						// (uint8_t)(value & 0xFF) };     
	
	buffer[0] = (uint8_t)((value >> 8) & 0xFF); // Upper 8-bits
	buffer[1] = (uint8_t)((value ) & 0xFF); // Lower 8-bits


    return bI2C_WriteBytes(reg, buffer, 2);                       // Register
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
bool wireReadRegister(uint8_t reg, uint16_t *value)
{
	uint8_t buffer[2];
	bool success;
		
	success = bI2C_ReadBytes(reg, buffer, 2);

    // Shift values to create properly formed integer
	if (success)
		*value = (((uint16_t)buffer[0] << 8) | buffer[1]); // can maybe be replaced by just a cast into uint16_t ??

    return success;
}


/**************************************************************************/
/*!
    @brief  Configures to INA219 to be able to measure up to 32V and 1A
            of current.  Each unit of current corresponds to 40uA, and each
            unit of power corresponds to 800?W. Counter overflow occurs at
            1.3A.
    @note   These calculations assume a 0.1 ohm resistor is present
*/
/**************************************************************************/
void set237Calibration_1A(void)
{
  // RSHUNT = 0.1			(Resistor value in ohms)


 // MaxExpected_I = 1.0A


  // CurrentLSB = MaxExpected_I/32767
  // CurrentLSB = 0.000030518             (30.5?A per bit)




  // 5. Compute the calibration register
  // Shunt_cal = constant * CurrentLSB * Rshunt
  // Cal = 819.2 * 10^6 * 30.518*10^-6 * 0.1

   //_calValue = 2500;
	
	// Cal = 819.2 * 10^6 * 30.518*10^-6 * 0.05

   _calValue = 1250;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0008 (800?W per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.31068A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // ... In this case, we're good though since Max_Current is less than MaxPossible_I
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.131068V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.31068 * 32V
  // MaximumPower = 41.94176W

  // Set multipliers to convert raw current/power values
  _currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
  _powerMultiplier_mW = 0.8f;   // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(REG_SHUNT_CAL, _calValue);

  // Set Config register to take into account the settings above
//  uint16_t config;
//  = CONFIG_BVOLTAGERANGE_32V |
//                    CONFIG_GAIN_8_320MV |
//                    CONFIG_BADCRES_12BIT |
//                    CONFIG_SADCRES_12BIT_1S_532US |
//                    CONFIG_MODE_SANDBVOLT_CONTINUOUS;
//  wireWriteRegister(REG_CONFIG, config);
}


/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
    @return the raw bus voltage reading
*/
/**************************************************************************/
int16_t getBusVoltage_raw() {
  uint16_t value = 0;
  wireReadRegister(REG_VBUS, &value);


  return (int16_t)(value);
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
    @return the raw shunt voltage reading
*/
/**************************************************************************/
int16_t getShuntVoltage_raw() {
  uint16_t value = 0;
  // wireReadRegister(REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
    @return the raw current reading
*/
/**************************************************************************/
int16_t getCurrent_raw() {
  uint16_t value = 0;


  wireWriteRegister(REG_SHUNT_CAL, _calValue);

  // Now we can safely read the CURRENT register!
  wireReadRegister(REG_CURRENT, &value);

  return (int16_t)value;
}

bool getCurrent_raw_(int16_t * rawValue) {
  uint16_t value = 0;
  bool success = true ;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  success = success & wireWriteRegister(REG_SHUNT_CAL, _calValue);

  // Now we can safely read the CURRENT register!
  success = success & wireReadRegister(REG_CURRENT, &value);

  if (success)
	  *rawValue = (int16_t)value;

  return success;
}

/**************************************************************************/
/*!
    @brief  Gets the raw power value (16-bit signed integer, so +-32767)
    @return raw power reading
*/
/**************************************************************************/
int16_t getPower_raw() {
  uint16_t value = 0;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wireWriteRegister(REG_SHUNT_CAL, _calValue);

  // Now we can safely read the POWER register!
  wireReadRegister(REG_POWER, &value);

  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV (so +-327mV)
    @return the shunt voltage converted to millivolts
*/
/**************************************************************************/
float getShuntVoltage_mV() {
  int16_t value = 0;
  value = getShuntVoltage_raw();
  return value * 0.01;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
    @return the bus voltage converted to volts
*/
/**************************************************************************/
float getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
	
  return (float)value * 0.003125;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
    @return the current reading convereted to milliamps
*/
/**************************************************************************/
float getCurrent_mA() {
  float valueDec = getCurrent_raw();
 // valueDec /= _currentDivider_mA;
  return valueDec * 0.000030518*1000;
}

bool getCurrentAmps(float * amps) {
  int16_t valueRaw;
  float valueDec;
  bool success = getCurrent_raw_(&valueRaw);
  valueDec = valueRaw;
  valueDec /= _currentDivider_mA;
  valueDec /= 1000.0f;
  if (success)
	  *amps = valueDec;
  return success;
}

/**************************************************************************/
/*!
    @brief  Gets the power value in mW, taking into account the
            config settings and current LSB
    @return power reading converted to milliwatts
*/
/**************************************************************************/
float getPower_mW() {
  float valueDec = getPower_raw();
  valueDec *= _powerMultiplier_mW;
  return valueDec;
}


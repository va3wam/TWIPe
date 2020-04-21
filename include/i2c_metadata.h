/************************************************************************************************************************************
 * @file i2c_metadata.h
 * @author va3wam
 * @brief Define all I2C related items
 * @version 0.1
 * @date 2020-04-21
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ---------------------------------------------------------------------------------------------------------------
 * 0.0.1   2020-04-21 Program created 
 ************************************************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief I2C bus0 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_bus0_speed 400000 // Define speed of I2C bus 2. Note 400KHz is the upper speed limit for ESP32 I2C
#define I2C_bus0_SDA 22 // Define pin on the board used for Serial Data Line (SDA) for I2C bus 0
#define I2C_bus0_SCL 14 // Define pin on the board used for Serial Clock Line (SCL) for I2C bus 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief I2C bus1 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_bus1_speed 400000 // Define speed of I2C bus 2. Note 400KHz is the upper speed limit for ESP32 I2C
#define I2C_bus1_SDA 17 // Define pin on the board used for Serial Data Line (SDA) for I2C bus 1
#define I2C_bus1_SCL 21 // Define pin on the board used for Serial Clock Line (SCL) for I2C bus 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief I2C device addresses 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MPU6050_I2C_ADD 0x68 // GY521 I2C address
#define leftOLED_I2C_ADD 0x3D // OLED used for robot's left eye I2C adddress
#define rightOLED_I2C_ADD 0x3C // OLED used for robot's right eye I2C address
#define LCD_I2C_ADD 0x3F // LED I2C address. Some models have address 0x38
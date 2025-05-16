///////////////////////////////////////////////////////////////////////////////
// Header file for MPU 6050 class
//  Used to read and write data.
//
//  Jose Pagan
//  04/10/2025
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <Arduino.h>
#include <Wire.h>

#define  GYRO_REG_SIZE         6
#define  ACCEL_REG_SIZE        6

#define ACCEL_SENSITIVITY   16384.0
#define  GYRO_SENSITIVITY     131.0

#define ANGLE_CONVERSION    ( 180.0 / PI )

// Power Macros
#define POWER_STRING( a )  \
    ( mpu6050::POWER_SLEEP == (a) ) ? "SLEEP" : \
    ( mpu6050::POWER_WAKE == (a) )  ? "WAKE" : "FAILED"
    
#define POWER_STATE( a ) ( \
    ( mpu6050::POWER_SLEEP == (a) ) ? mpu6050::POWER_SLEEP : \
    ( mpu6050::POWER_WAKE == (a) )  ? mpu6050::POWER_WAKE : mpu6050::POWER_FAILED )


// Temperature macros
#define TEMPERATURE_STRING( a ) \
    ( mpu6050::TEMPERATURE_DISABLE == (a) ) ? "DISABLE" : \
    ( mpu6050::TEMPERATURE_ENABLE == (a) )  ? "ENABLE" : "FAILED"

#define TEMPERATURE_STATE( a ) ( \
    ( mpu6050::TEMPERATURE_DISABLE == (a) ) ? mpu6050::TEMPERATURE_DISABLE : \
    ( mpu6050::TEMPERATURE_ENABLE == (a) )  ? mpu6050::TEMPERATURE_ENABLE : mpu6050::TEMPERATURE_FAILED )


class mpu6050
{
  public:
    // Power Macros
    typedef enum {
        MPU6050_SUCCESS = 0,
        MPU6050_ERROR,
        MPU6050_INVALID_REGISTER,
        MPU6050_INVALID_POINTER,
        MPU6050_INVALID_SIZE,
        MPU6050_TEMP_DISABLED,
        MPU6050_INVALID_TEMP_STATE,
        MPU6050_I2C_ERROR,
        MPU6050_I2C_READ,
        MPU6050_I2C_WRITE,
        MPU6050_MAX_ERROR
    } ERR_CODE;

    typedef enum  {
        POWER_FAILED = -1,
        POWER_WAKE,
        POWER_SLEEP,
        MAX_POWER_STATE
    } POWER_STATE;

    typedef enum  {
        TEMPERATURE_FAILED = -1,
        TEMPERATURE_DISABLE,
        TEMPERATURE_ENABLE,
        MAX_TEMPERATURE_STATE
    } TEMPERATURE_STATE;


    mpu6050( uint8_t ado = 0 );     // Constructor
    ~mpu6050();                     // Destructor

    // Read the sensor ID for the "WhoAmI" register
    u_char              getSensorID(void);

    // Reset the Chip
    ERR_CODE            sensorReset( void );

    // Chip power state methods 
    //   Accessor - returns current state
    POWER_STATE         getPowerState(void);
    //   Mutator - set new state and return previous state
    ERR_CODE            setPowerState(POWER_STATE state);


    //  Accessor/Mutator - temperature state
    TEMPERATURE_STATE   getTempState( void );
    ERR_CODE            setTempState(TEMPERATURE_STATE state);
    // Read the Temperature value
    ERR_CODE            getTemperature( float *temp );

    // Read the Gyro values
    ERR_CODE            getGyroValues( float *x, float *y, float *z );

    // Read the Gyro values
    ERR_CODE            getAccelValues( float *x, float *y, float *z );

  private:
    bool                isValidRegister( uint8_t reg );

    void                init(void);
    ERR_CODE            gyroCalibrate( void );
    ERR_CODE            accelCalibrate( void );

    ERR_CODE             readI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size);
    ERR_CODE            writeI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size);

    u_char              readSensorID(void);
    ERR_CODE            readPowerRegister(void);
    ERR_CODE            readTemperatureRegister( void );
    ERR_CODE            readGyroRegisters( void );
    ERR_CODE            readAccelRegisters( void );
//    ERR_CODE            readAllRegisters( void );

    const uint32_t      I2C_CLOCK_SPEED     = 400000;       // MPU6050 I2C address
    const uint8_t       I2C_BASE            = 0x68;         // MPU6050 I2C address
    uint8_t             sensorADO           = 0;            // reflects ADO

    uint8_t             addressI2C          = 0xff;         // Computed Address
    u_char              sensorID            = 0xff;         // ID of this sensor (Who-Am-I)

    // Components of the Power Management 1 register ( )
    u_char              powerReg           =  0xff ;     // Store Power management register #1
    POWER_STATE         powerState          = POWER_FAILED; // Power management register #1
    TEMPERATURE_STATE   temperatureState    = TEMPERATURE_DISABLE; // Temperature state
    uint8_t             clockSource         = 0;            // Clock source
        // 0 (default) = Internal 8MHz oscillator
        // 1 = PLL with x-axis gyroscope reference
        // 2 = PLL with y-axis gyroscope reference
        // 3 = PLL with z-axis gyroscope reference
        // 4 = PLL with external 32.768kHz reference
        // 5 = PLL with external 19.2MHz reference
        // 6 = PLL with external 48MHz reference
        // 7 = Stop the clock

    // Raw temperature value
    uint16_t            tempRaw                     = 0;                // Raw data from the sensor
    float               temperature                 = 0.0;              // Raw temperature value

    // Gyro values
    float               GyroSensitivity             = GYRO_SENSITIVITY; // Scale factor for the gyro
    int16_t             GyroSensitivityFS           = 0;                // Full scale range
    uint8_t             gyroRaw[GYRO_REG_SIZE]      = { 0 };            // Raw data from the sensor
    int16_t             GyroX                       = 0;
    int16_t             GyroY                       = 0;
    int16_t             GyroZ                       = 0;

    float               GyroX_cal                   = 0;
    float               GyroY_cal                   = 0;
    float               GyroZ_cal                   = 0;

    // Accelerometer values
    float               AccelSensitivity            = ACCEL_SENSITIVITY; // Scale factor for the accelerometer
    uint8_t             AccelSensitivityFS          = 0;                 // Full scale range
    uint8_t             accelRaw[ACCEL_REG_SIZE]    = { 0 };             // Raw data from the sensor
    int16_t             AccelX                      = 0;
    int16_t             AccelY                      = 0;
    int16_t             AccelZ                      = 0;

    float               AccelX_cal                  = 0;
    float               AccelY_cal                  = 0;
    float               AccelZ_cal                  = 0;

    float               AccelX_err                  = 0;
    float               AccelY_err                  = 0;
    float               AccelZ_err                  = 0;

    float               AccelXOffset                = 0;
    float               AccelYOffset                = 0;
    float               AccelZOffset                = 0;
    float               AccelXScale                 = 0;
    float               AccelYScale                 = 0;
    float               AccelZScale                 = 0;
    float               AccelXOffsetScale           = 0;
    float               AccelYOffsetScale           = 0;
    float               AccelZOffsetScale           = 0;
};


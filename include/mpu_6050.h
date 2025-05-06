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
        MPU6050_I2C_ERROR,
        MPU6050_I2C_READ,
        MPU6050_I2C_WRITE,
        MPU6050_MAX_ERROR
    } ERR_CODE;

    typedef enum  {
        POWER_FAILED = -1,
        POWER_SLEEP,
        POWER_WAKE,
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

    ERR_CODE             readI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size);
    ERR_CODE            writeI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size);

    // Read the sensor ID for the "WhoAmI" register
    u_char              getSensorID(void);

    // Reset the Chip
    ERR_CODE            sensorReset( void );

    // Chip power state methods 
    //   Accessor - returns current state
    POWER_STATE         getPowerState(void);
    //   Mutator - set new state and return previous state
    ERR_CODE            setPowerState(POWER_STATE state);


    //   Mutator - set new state and return previous state
    TEMPERATURE_STATE   getTempState( void );
    ERR_CODE            setTempState(TEMPERATURE_STATE state);
    // Read the Temperature
    ERR_CODE            getTemperature( uint16_t *temp );

//        void gyroSignals(void);
//        void setFilter(uint8_t filter);
//        void setSensitivity(uint8_t sensitivity);
//        uint16_t getSensorID(void);
//        float getRateRoll(void);
//        float getRatePitch(void);
//        float getRateYaw(void);

  private:
    bool                isValidRegister( uint8_t reg );

    void                init(void);
    ERR_CODE            readPowerRegister(void);
    u_char              readSensorID(void);

    const uint32_t      I2C_CLOCK_SPEED     = 400000;       // MPU6050 I2C address
    const uint8_t       I2C_BASE            = 0x68;         // MPU6050 I2C address
    uint8_t             sensorADO           = 0;            // reflects ADO

    uint8_t             addressI2C          = 0xff;         // Computed Address
    u_char              sensorID            = 0xff;         // ID of this sensor (Who-Am-I)
    u_char              powerReg            = 0xff;         // ID of this sensor (Who-Am-I)
    POWER_STATE         powerState          = POWER_FAILED; // Power management register #1

    // Raw temperature value
    uint8_t             rawData[2]          = { 0 };        // Raw data from the sensor
    float               temperature         = 0.0;          // Raw temperature value
    TEMPERATURE_STATE   temperatureState    = TEMPERATURE_DISABLE; // Temperature state

    float               RateRoll            = 0;
    float               RatePitch           = 0;
    float               RateYaw             = 0;

};


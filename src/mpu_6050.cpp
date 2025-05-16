///////////////////////////////////////////////////////////////////////////////
// Source file for MPU 6050 class
//  Used to read and write data.
//
//  Jose Pagan
//  04/10/2025
///////////////////////////////////////////////////////////////////////////////

#include <mpu_6050.h>

#define  PWR_MGMT_1         0x6B
#define  PWR_MGMT_2         0x6C
#define  TEMP_H_OUT         0x41
#define  TEMP_L_OUT         0x42
#define  GYRO_REG_BASE      0x43
#define  ACCEL_REG_BASE     0x3B

#define  PWR_RESET_BIT      0x80
#define  PWR_SLEEP_BIT      0x40
#define  TEMP_DISABLE_BIT   0x08
#define  PWR_CLK_SEL_MASK   0x07

#define SERIAL_BAUD_RATE    115200

#define isValidTemperatureState( a )  \
    (    ( mpu6050::TEMPERATURE_DISABLE == (a) ) \
      || ( mpu6050::TEMPERATURE_ENABLE  == (a) ) \
    )


/***********************************************************************
**  Constructor
************************************************************************/
mpu6050::mpu6050( uint8_t ado )
{
    #ifdef JDEBUG
    Serial.printf( " INFO: MPU6050 Constructor\n" );
    #endif
    if( 1 < ado )
    { // ADO value is out of range
        Serial.printf( "ERROR: MPU6050 ADO value out of range (%d), using 0\n", ado );
        sensorADO = 0; // Set to device's default value
    }
    else
    {
        sensorADO = ado; // Set ADO value
    }

    addressI2C = I2C_BASE + sensorADO; // Compute the I2C address    

    // Initialize the device
    init(); 

}


/***********************************************************************
** Destructor
************************************************************************/
mpu6050::~mpu6050()
{
    // Destructor
    #ifdef JDEBUG
    Serial.printf( " INFO: MPU6050 Destructor\n" );
    #endif
    Wire1.end(); // Stop I2C communication

}


/***********************************************************************
** Public - return current power state
************************************************************************/
mpu6050::POWER_STATE
mpu6050::getPowerState()
{
    return powerState;
}


/***********************************************************************
** Return the sensor ID from the "WhoAmI" register
************************************************************************/
u_char
mpu6050::getSensorID(void)
{
    return sensorID;
}


/***********************************************************************
** Private. Validate register is in the range for ths device.
************************************************************************/
#define MPU6050_LOW_REG         0x0D
#define MPU6050_HIGH_REG        0x75
#define MPU6050_MAX_REG_BLOCK     20
// // //
bool
mpu6050::isValidRegister( uint8_t reg )
{
    bool retVal = true;

    if( (MPU6050_LOW_REG > reg) || (MPU6050_HIGH_REG < reg) )
    { // Register is out of range
        retVal = false;
        Serial.printf( "ERROR: MPU6050 register out of range (%#x)\n", reg );
    }

    return retVal;
}


/***********************************************************************
** Private function to read one or more I2C Registers
************************************************************************/
mpu6050::ERR_CODE
mpu6050::readI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size )
{
    ERR_CODE retCode = MPU6050_SUCCESS;
    uint8_t i2cErr = 0;
    uint8_t i2cReadSize = 0;

    if(  false == isValidRegister( reg ) )
    {
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: register out of range (%#x)\n", reg );
        retCode = MPU6050_INVALID_REGISTER;
    }
    else if( NULL == data )
    {
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: data pointer is NULL\n" );
        retCode = MPU6050_INVALID_POINTER;
    }
    else if( 0 == size )
    {
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: size is zero\n" );
        retCode = MPU6050_INVALID_SIZE;
    }
    else if( size > MPU6050_MAX_REG_BLOCK )
    {
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: read size is too large (%d)\n", size );
        retCode = MPU6050_INVALID_SIZE;
    }
    else if( Wire1.beginTransmission( addressI2C ), // MPU6050 I2C address                
             ( 1 != Wire1.write( reg ) )            // register offset
           )
    { // the write failed to send one byte
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: write failed\n" );
        retCode = MPU6050_I2C_WRITE;
        // Hail Mary! IT DOES NOT MATTER IF THE endTransmission() CALL FAILS
        Wire1.endTransmission(); // to be complete befor exiting 
    }
    else if( 0 != ( i2cErr = Wire1.endTransmission() ) )
    {
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: endTransmission() failed with error (%d)\n", i2cErr );
        retCode = MPU6050_I2C_WRITE;
    }
    else if(  0 == ( i2cReadSize = Wire1.requestFrom( addressI2C, size ) ) )
    { // Request the data from the MPU6050
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: request read failed with error (%d)\n", i2cReadSize );
        retCode = MPU6050_I2C_ERROR;
    }
/*
    else if( 0 == i2cReadSize )
    { // No data was read
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: no data read\n" );
        retCode = MPU6050_I2C_READ;
    }
*/
    else if( i2cReadSize != size )
    { // Requested size is wrong
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: read size mismatch (%d != %d)\n", i2cReadSize, size );
        retCode = MPU6050_I2C_READ;
    }
    else
    { // Read the I2C registers

        while( Wire1.available() < i2cReadSize );

        for( uint8_t i = 0; i < i2cReadSize; i++ )
        {
            data[i] = Wire1.read();
            #ifdef JDEBUG
            Serial.printf( " INFO: MPU6050 readI2cRegisters: read register(%d): %#x\n"
                           , reg + i
                           , data[i]
                         );
            #endif
        }

        retCode = MPU6050_SUCCESS;
    }

    return retCode;
}


/***********************************************************************
** Private function to write one or more I2C Registers
************************************************************************/
mpu6050::ERR_CODE
mpu6050::writeI2cRegisters( uint8_t reg, uint8_t *data, uint8_t size )
{
    ERR_CODE retCode = MPU6050_SUCCESS;

    if(  false == isValidRegister( reg ) )
    {
        Serial.printf( "ERROR: MPU6050 register out of range (%#x)\n", reg );
        retCode = MPU6050_INVALID_REGISTER;
    }
    else if( NULL == data )
    {
        Serial.printf( "ERROR: MPU6050 data pointer is NULL\n" );
        retCode = MPU6050_INVALID_POINTER;
    }
    else if( 0 == size )
    {
        Serial.printf( "ERROR: MPU6050 size is zero\n" );
        retCode = MPU6050_INVALID_SIZE;
    }
    else if( size > MPU6050_MAX_REG_BLOCK )
    {
        Serial.printf( "ERROR: MPU6050 write size is too large (%d)\n", size );
        retCode = MPU6050_INVALID_SIZE;
    }
    else
    { // Logic to write the register(s)
        Wire1.beginTransmission( addressI2C );  // MPU6050 start with I2C address

        if( 1 != Wire1.write( reg ) )           // write register offset
        {
            Serial.printf( "ERROR: MPU6050 set write address failed \n" );
            retCode = MPU6050_I2C_WRITE;
        }
        else if( Wire1.write( data, size ) != size )
        { // Writing data failed
            Serial.printf( "ERROR: MPU6050 write register(s) failed \n" );
            retCode = MPU6050_I2C_WRITE;
        }
        else
        { // Write was successful
            #ifndef JDEBUG
            Serial.printf( " INFO: MPU6050 writeI2cRegisters: wrote register(s): %#x, data, %#x, size: %d\n"
                           , reg
                           , data[0]
                           , size
                         );
            #endif
        }

        { // Always end an I2C transmission
            uint8_t i2cErr = Wire1.endTransmission();
            if( 0 != i2cErr )
            { // Check final result
                Serial.printf( "ERROR: MPU6050 write register(s) failed with error (%d)\n", i2cErr );
                if( !retCode == MPU6050_SUCCESS )
                { // Only set the error if it was SUCCESS
                    retCode = MPU6050_I2C_ERROR;
                }

            }

        }


    }

    return retCode;
}


/***********************************************************************
** Private function to read the sensor ID from the "WhoAmI" register
************************************************************************/
#define SENSOR_WHO_AM_I     0x75
// // //
u_char
mpu6050::readSensorID(void)
{
    mpu6050::ERR_CODE  errCode = readI2cRegisters( (uint8_t)SENSOR_WHO_AM_I, (uint8_t *)&sensorID, (uint8_t)1 );

    if( MPU6050_SUCCESS != errCode )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 readSensorID failed with error [%d]\n", errCode );
        sensorID = 0xFF; // Set to invalid value
    }
    // else
    // { // Read was successful
    //     Serial.printf( " INFO: MPU6050 readSensorID -- ID: %#x\n", sensorID );
    // }

    return sensorID;
}


/***********************************************************************
**
************************************************************************/
// Public - reset the power state
mpu6050::ERR_CODE
mpu6050::setPowerState(POWER_STATE newState)
{
    ERR_CODE retCode = MPU6050_SUCCESS;

    #ifdef JDEBUG
    Serial.printf( " INFO: MPU6050 setPowerState() -- new power state: (%d) [%s]\n"
                   ,newState
                   ,POWER_STRING( newState )
                 );

    Serial.printf( " INFO: MPU6050 setPowerState() -- current power state: (%d) [%s]\n"
                   ,powerState
                   ,POWER_STRING( powerState )
                 );
    #endif

    if( newState == powerState)
    { // The current state matches the proposed (new) state
        Serial.printf( " WARN: MPU6050 setPowerState() -- power state unchanged, (%s)\n"
                       ,POWER_STRING( newState )
                    );
    }
    else if( MPU6050_SUCCESS != ( retCode = readI2cRegisters( (uint8_t)PWR_MGMT_1, (uint8_t *)&powerReg, (uint8_t)1 ) ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 setPowerState() failed to read power register with error [%d]\n", retCode );
        powerState = POWER_FAILED; // Set to invalid value
    }
    else
    {  // Read was successful

        // Set the new power register values:
        powerState = ( powerReg & PWR_SLEEP_BIT ) ? POWER_SLEEP : POWER_WAKE;
        temperatureState = ( powerReg & TEMP_DISABLE_BIT ) ? TEMPERATURE_DISABLE : TEMPERATURE_ENABLE;
        clockSource = ( powerReg & PWR_CLK_SEL_MASK );

        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 setPowerState() -- read power reg: (%#x), POWER [%s], Temperature [%s], Clock source: (%#x)\n"
                   ,powerReg
                   ,POWER_STRING( powerState )
                   ,TEMPERATURE_STRING( temperatureState )
                   ,clockSource
                 );
        #endif

        if( POWER_SLEEP == newState )
        { // Set the new power value and send to MPU6050
            powerReg |= PWR_SLEEP_BIT;
            powerState = POWER_SLEEP;
        }
        else
        { 
            powerReg &= ~PWR_SLEEP_BIT;
            powerState = POWER_WAKE;
        }

        if( 0 != ( retCode = writeI2cRegisters( (uint8_t)PWR_MGMT_1, (uint8_t *)&powerReg, (uint8_t)1 ) ) )
        { // Write failed
            Serial.printf( "ERROR: MPU6050 setPowerState() failed to write power register with error [%d]\n", retCode );
            powerState = POWER_FAILED; // Set to invalid value
            temperatureState = TEMPERATURE_FAILED;
        }
        else
        { // Write was successful
            #ifdef JDEBUG
            Serial.printf( " INFO: MPU6050 setPowerState() -- NEW power reg: (%#x), POWER [%s], Temperature [%s]\n"
                           ,powerReg
                           ,POWER_STRING( powerState )
                           ,TEMPERATURE_STRING( temperatureState )
                         );
            #endif
        }

    }

    return retCode;
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


/***********************************************************************
** Private:
**   read the power register and write result to powerReg
**  Also, compute POWER STATE and TEMPERATURE_STATE
**   
************************************************************************/
//mpu6050::POWER_STATE
mpu6050::ERR_CODE
mpu6050::readPowerRegister( void )
{ // read the value of the power register 

    ERR_CODE errRet = readI2cRegisters( (uint8_t)PWR_MGMT_1, (uint8_t *)&powerReg, (uint8_t)1 );

    if( MPU6050_SUCCESS != errRet )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 readPowerRegister failed with error [%d]\n", errRet );
        powerState = POWER_FAILED; // Set to invalid value
        temperatureState = TEMPERATURE_FAILED; // Set to invalid value
    }
    else
    { // Read was successful
        powerState = ( powerReg & PWR_SLEEP_BIT ) ? POWER_SLEEP : POWER_WAKE;
        temperatureState = ( powerReg & TEMP_DISABLE_BIT ) ? TEMPERATURE_DISABLE : TEMPERATURE_ENABLE;
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 readPowerRegister Value: [%#x], State: [%s], Temp: [%s]\n"
                       , powerReg
                       , POWER_STRING( powerState )
                       , TEMPERATURE_STRING( temperatureState )
                     );
        #endif
    }

    return errRet;
}



/***********************************************************************
**
************************************************************************/
mpu6050::TEMPERATURE_STATE
mpu6050::getTempState( void )
{
    return temperatureState;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::setTempState(TEMPERATURE_STATE newState )
{
    ERR_CODE retVal = MPU6050_SUCCESS;

    if( ! isValidTemperatureState( newState ) )
    { // The current state matches the proposed (new) state
        Serial.printf( " WARN: MPU6050 setTempState -- invalid temperature state (%d)\n"
                       ,newState
                    );
        retVal = MPU6050_INVALID_TEMP_STATE;
    }
    else if( MPU6050_SUCCESS != ( retVal = readPowerRegister() ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 setTempState -- failed to read power register with error [%d]\n", retVal );
        temperatureState = TEMPERATURE_FAILED; // Set to invalid value
    }
    else
    {

        // Reach here and the power management 1 register has been refreshed
        if( TEMPERATURE_ENABLE == newState )
        { // Set the new power value and send to MPU6050
            powerReg &= ~TEMP_DISABLE_BIT;
        }
        else if( TEMPERATURE_DISABLE == newState )
        { // Set the new power value and send to MPU6050
            powerReg |= TEMP_DISABLE_BIT;
        }

        // Set the new temperature state in the power management 1 register
        if( Wire1.beginTransmission( addressI2C )
            ,1 != Wire1.write( PWR_MGMT_1 )
          )
        { // the write failed to send one byte
            Serial.printf( "ERROR: setTempState -- failed to select the MPU6050 power register\n" );
            retVal = MPU6050_I2C_WRITE; // Set to invalid value
        }
        else if( 1 != Wire1.write( powerReg ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: setTempState -- failed to set temperature state on the MPU6050\n" );
            retVal = MPU6050_I2C_WRITE; // Set to invalid value
        }
        else
        {  // Write successfully sent temperature sensor offset
            Serial.printf( " INFO: MPU6050 setTempState -- state set to %s\n", TEMPERATURE_STRING( temperatureState ) );
            retVal = MPU6050_SUCCESS;
        }

        // Always try to end the transmission
        int cmdRet = Wire1.endTransmission();

        if( 0 != cmdRet )
        { // Report if ending transmission fails
            Serial.printf( "ERROR: MPU6050 setTempState -- call to endTransmission() failed with error [%d]\n", retVal );
            retVal = MPU6050_I2C_ERROR; // Set to invalid value
        }
        else
        { // Read was successful
            temperatureState = newState;
            #ifdef JDEBUG
            Serial.printf( " INFO: MPU6050 setTempState() -- read power reg: (%#x) [%s], Temperature [%s]\n"
                           ,powerReg
                           ,POWER_STRING( powerState )
                           ,TEMPERATURE_STRING( temperatureState )
                         );
            #endif
        }

    }

    return retVal;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::getTemperature( float *temp )
{
    ERR_CODE retVal = MPU6050_INVALID_POINTER;

    if( NULL == temp )
    { // Pointer is NULL
        Serial.printf( "ERROR: MPU6050 getTemperature() -- pointer is NULL\n" );
    }
    else if( MPU6050_SUCCESS != ( retVal = readTemperatureRegister() ) )
    { // Reading of registers failed
        Serial.printf( "ERROR: MPU6050 getTemperature() failed with error [%d]\n", retVal );
        *temp = -99;
    }
    else
    { // Read was successful
        *temp = (float)( ( temperature / 340.0 ) + 36.53 ); // Convert to degrees C
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 getTemperature -- Temp: %f\n", *temp );
        #endif
    }

    return retVal;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::readTemperatureRegister( void )
{
    ERR_CODE retVal = MPU6050_SUCCESS;

    if ( POWER_WAKE != powerState )
    { // Device is not in the correct state
        Serial.printf( "ERROR: MPU6050 readTemperatureRegister sensor power state is %s\n"
                       , POWER_STRING( powerState)
                     );
        retVal = MPU6050_ERROR;
    }
    else if ( TEMPERATURE_ENABLE != temperatureState )
    { // Temperature sensor is not enabled
        Serial.printf( "ERROR: MPU6050 readTemperatureRegister temperature state %s, not enabled\n"
                       ,TEMPERATURE_STRING( temperatureState )
                     );
        retVal = MPU6050_TEMP_DISABLED;
    }
    else
    { // Power and temperature are enabled

        uint8_t readData[2] = { 0 };
        // Read the temperature registers
        if( MPU6050_SUCCESS != ( retVal = readI2cRegisters( (uint8_t)TEMP_H_OUT, (uint8_t *)readData, (uint8_t)2 ) ) )
        { // Read failed
            Serial.printf( "ERROR: MPU6050 readTemperatureRegister Hi readI2cRegisters failed with error [%d]\n", retVal );
            temperatureState = TEMPERATURE_FAILED; // Set to invalid value
        }
        else
        { // Reads were successful
            temperature = (int16_t)( (readData[0] << 8) | readData[1] );
            #ifdef DEBUG
            Serial.printf( " INFO: MPU6050 readTemperatureRegister -- read registers (%02#x:%02#x), Temp: [%f] \n"
                           , readData[0], readData[1], temperature
                         );
 
            #endif
        }

    }

    return retVal;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::getGyroValues( float *x, float *y, float *z )
{
    ERR_CODE retVal = MPU6050_ERROR;

    if( NULL == x || NULL == y || NULL == z )
    { // Pointer is NULL
        Serial.printf( "ERROR: MPU6050 getGyroValues() -- pointer is NULL\n" );
        retVal = MPU6050_INVALID_POINTER;
    }
    else if( MPU6050_SUCCESS != ( retVal = readGyroRegisters() ) )
    { // Reading of registers failed
        Serial.printf( "ERROR: MPU6050 getGyroValues() failed with error [%d]\n", retVal );
    }
    else
    { // Read was successful
        *x = ( GyroX / GyroSensitivity ) - GyroX_cal;
        *y = ( GyroY / GyroSensitivity ) - GyroY_cal;
        *z = ( GyroZ / GyroSensitivity ) - GyroZ_cal;

        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 getGyroValues -- GyroX: %04#x ( %f/ %f = %f - %f )  %f  "
                       , GyroX, (float)GyroX, GyroSensitivity, ( GyroX / GyroSensitivity ), GyroX_cal, *x
                     );
        Serial.printf( "GyroY: %04#x (%f)  "
                       , (int16_t)*y, *y
                     );
        Serial.printf( " GyroZ: %04#x (%f)\n"
                       , (int16_t)*z, *z
                     );
        #endif
    }

    return retVal;
}



/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::readGyroRegisters( void )
{
    ERR_CODE retVal = readI2cRegisters( (uint8_t)GYRO_REG_BASE, (uint8_t *)gyroRaw, (uint8_t)GYRO_REG_SIZE );

    if( MPU6050_SUCCESS != retVal )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 readGyroRegisters failed with error [%d]\n", retVal );
    }
    else
    { // Read was successful, swap the bytes
        GyroX = (int16_t)( ( gyroRaw[0] << 8 ) | gyroRaw[1] );
        GyroY = (int16_t)( ( gyroRaw[2] << 8 ) | gyroRaw[3] );
        GyroZ = (int16_t)( ( gyroRaw[4] << 8 ) | gyroRaw[5] );

        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 readGyroRegisters -- GyroX=%04#x (%d),  GyroY=%04#x (%d),  GyroZ=%04#x (%d)\n"
                       , (uint16_t)GyroX, GyroX, (uint16_t)GyroY, GyroY, (uint16_t)GyroZ, GyroZ
                     );
        #endif
    }

    return retVal;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::mpu6050::getAccelValues( float *x, float *y, float *z )
{
    ERR_CODE retVal = MPU6050_ERROR;

    if( NULL == x || NULL == y || NULL == z )
    { // Pointer is NULL
        Serial.printf( "ERROR: MPU6050 getAccelValues() -- pointer is NULL\n" );
        retVal = MPU6050_INVALID_POINTER;
    }
    else if( MPU6050_SUCCESS != ( retVal = readAccelRegisters() ) )
    { // Reading of registers failed
        Serial.printf( "ERROR: MPU6050 getAccelValues() failed with error [%d]\n", retVal );
    }
    else
    { // Read was successful
        *x = ( AccelX / AccelSensitivity ) - AccelX_cal;
        *y = ( AccelY / AccelSensitivity ) - AccelY_cal;
        *z = ( AccelZ / AccelSensitivity );
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 getAccelValues -- AccelX: %04#x, AccelY: %04#x, AccelZ: %04#x\n"
                       , (float)*x, (float)*y, (float)*z
                     );
        #endif
    }

    return retVal;
}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::readAccelRegisters( void )
{
    ERR_CODE retVal = readI2cRegisters( (uint8_t)ACCEL_REG_BASE, (uint8_t *)accelRaw, (uint8_t)ACCEL_REG_SIZE );

    if( MPU6050_SUCCESS != retVal )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 readAccelRegisters failed with error [%d]\n", retVal );
    }
    else
    { // Read was successful, swap the bytes
       AccelX =  (int16_t)( (accelRaw[0] << 8) | accelRaw[1] );
       AccelY =  (int16_t)( (accelRaw[2] << 8) | accelRaw[3] );
       AccelZ =  (int16_t)( (accelRaw[4] << 8) | accelRaw[5] );
  
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 readAccelRegisters -- AccelX: %04#x, AccelY: %04#x, AccelZ: %04#x \n"
                       , AccelX, AccelY, AccelZ
                     );
        #endif
    }
    return retVal;

}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
mpu6050::sensorReset( void )
{
    ERR_CODE retVal = MPU6050_SUCCESS;

    // Refresh the current Power management Register value
    if( MPU6050_SUCCESS != readPowerRegister() )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 sensorReset failed with error [%d]\n", powerState );
    }
    else
    { // Success on reding the MPU6050 power register
        if( Wire1.beginTransmission( addressI2C ), // MPU6050 I2C address, returns nothing
             1 != Wire1.write( PWR_MGMT_1 ) // write register offset, returns 1 on success
          )
        { // the write failed to send the address byte
            Serial.printf( "ERROR: I2C failed to send power byte to MPU6050\n" );
            retVal = MPU6050_I2C_WRITE;
        }
        else if( 1 != Wire1.write( powerReg | PWR_RESET_BIT ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to set RESET state on the MPU6050\n" );
            retVal = MPU6050_I2C_WRITE;
        }

        // Always try to end the transmission
        {
            int cmdRet = Wire1.endTransmission();

            if( 0 != cmdRet )
            { // Report if ending transmission fails
                Serial.printf( "ERROR: MPU6050 sensorReset failed with error [%d]\n", retVal );
                retVal = MPU6050_I2C_ERROR; // Set to invalid value
            }
            else
            { // Read was successful
                init(); // Reinitialize and calibrate the sensor
            }
        }

    }

    return retVal;
}


/***********************************************************************
** Function to calibrate the Accelerometers on the MPU6050 sensor board
************************************************************************/
#define ACCEL_CAL_COUNT     1050

mpu6050::ERR_CODE
mpu6050::accelCalibrate( void )
{  // Accel Values
  ERR_CODE retVal = MPU6050_SUCCESS;

  int32_t AccelX_sum = 0;
  int32_t AccelY_sum = 0;
  int32_t AccelZ_sum = 0;

  // Gyro Calibration
  Serial.printf( " INFO: MPU6050 Accelerometer Calibration ." );
  unsigned long prevTime = millis();

  // Read accelerometer values 200 times
  for( int c=0; c < ACCEL_CAL_COUNT; c++ )
  {  // read a number of values and take the average
    if(c % 125 == 0)
    {
        Serial.printf(".");             //Print a dot on the LCD every 125 readings
    }

    readAccelRegisters( );              // Read the accelerometer registers
    // Sum all readings
    AccelX_sum +=  AccelX;
    AccelY_sum +=  AccelY;
    AccelZ_sum +=  AccelZ;

    delay(3);                                         //Delay 3us to simulate the 250Hz program loop
  }

  //Divide each sum by ACCEL_CAL_COUNT to get the average
  AccelX_sum /= ACCEL_CAL_COUNT;
  AccelY_sum /= ACCEL_CAL_COUNT;
  AccelZ_sum /= ACCEL_CAL_COUNT;

  AccelX_cal = (float)( AccelX_sum / AccelSensitivity ); // Divide the sum by 2000 to get the average offset
  AccelY_cal = (float)( AccelY_sum / AccelSensitivity ); // Divide the sum by 2000 to get the average offset
  AccelZ_cal = (float)( AccelZ_sum / AccelSensitivity ); // Divide the sum by 2000 to get the average offset

  AccelX_err +=  ANGLE_CONVERSION * atan(      (AccelY_cal) / sqrt( pow( (AccelX_cal), 2) + pow((AccelZ_cal), 2) ) );
  AccelY_err +=  ANGLE_CONVERSION * atan( -1 * (AccelX_cal) / sqrt( pow( (AccelY_cal), 2) + pow((AccelZ_cal), 2) ) );
//  AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//  AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));

  unsigned long elapsedTime = ( millis() - prevTime );

  Serial.printf( "   Time: (%u [ms]) %f [s]\n INFO: MPU6050 Accel Calibration -- AccelX_cal: %f, AccelY_cal: %f, AccelZ_cal: %f\n"
                 ,elapsedTime ,elapsedTime/1000.0, AccelX_cal, AccelY_cal, AccelZ_cal
               );

  return retVal;
}


/***********************************************************************
** Function to calibrate the Gyros on the MPU6050 sensor board
************************************************************************/
#define GYRO_CAL_COUNT  1050

mpu6050::ERR_CODE
mpu6050::gyroCalibrate( void )
{  // Gyro Values
  ERR_CODE retVal = MPU6050_SUCCESS;

  // Gyro Calibration
  Serial.printf( " INFO: MPU6050 Gyro Calibration ." );
  unsigned long prevTime = millis();

  int32_t gyroX_sum = 0;
  int32_t gyroY_sum = 0;
  int32_t gyroZ_sum = 0;

  for (int cal_int = 0; cal_int < GYRO_CAL_COUNT ; cal_int ++)
  {  // read a number of values and take the average
      if(cal_int % 125 == 0)
      {
        Serial.printf(".");                 //Print a dot on the LCD every 125 readings
      }

      retVal = readGyroRegisters( );
      gyroX_sum += GyroX;                   //Sum all X readings
      gyroY_sum += GyroY;                   //Sum all Y readings
      gyroZ_sum += GyroZ;                   //Sum all Z readings

      delay(3);                             //Delay 3us to simulate the 250Hz program loop
  }

  unsigned long elapsedTime = ( millis() - prevTime );

  Serial.printf( "   Time: (%u [ms]) %f [s]\n INFO: MPU6050 Gyro Calibration -- SUM:  X=%#x,  Y=%#x,  Z=%#x \n"
                 , elapsedTime, (elapsedTime / 1000.0 ), gyroX_sum, gyroY_sum, gyroZ_sum
               );

  gyroX_sum /= GYRO_CAL_COUNT;                  //Divide the gyro_x_cal variable by count to get the avarage offset
  gyroY_sum /= GYRO_CAL_COUNT;                  //Divide the gyro_y_cal variable by count to get the avarage offset
  gyroZ_sum /= GYRO_CAL_COUNT;                  //Divide the gyro_z_cal variable by count to get the avarage offset

  Serial.printf( "   Time: (%u [ms]) %f [s]\n INFO: MPU6050 Gyro Calibration -- AVG:  X: %#x (%f) Y: %#x (%f), Z: %#x (%f) \n"
                 , elapsedTime, (elapsedTime / 1000.0 )
                 , gyroX_sum, (float)gyroX_sum
                 , gyroY_sum, (float)gyroY_sum
                 , gyroZ_sum, (float)gyroZ_sum
               );

  GyroX_cal = (float)( gyroX_sum / GyroSensitivity );               //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  GyroY_cal = (float)( gyroY_sum / GyroSensitivity );               //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  GyroZ_cal = (float)( gyroZ_sum / GyroSensitivity );               //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  Serial.printf( " INFO: MPU6050 Gyro Calibration -- CAL:  X=%#x (%f), Y=%#x (%f), Z=%#x (%f) \n"
                 , (int32_t)( gyroX_sum / GyroSensitivity ), GyroX_cal
                 , (int32_t)( gyroY_sum / GyroSensitivity ), GyroY_cal
                 , (int32_t)( gyroZ_sum / GyroSensitivity ), GyroZ_cal
               );

  return retVal;
}


/***********************************************************************
** Function to Initialize the MPU6050 sensor board
************************************************************************/
void 
mpu6050::init( void )
{
    // Set the Teensy I2c clock speed to 400kHz
    Wire1.setClock( I2C_CLOCK_SPEED );
    #ifdef JDEBUG
    Serial.printf( " INFO: MPU6050 Init I2C Clock speed: (%d)\n", I2C_CLOCK_SPEED );
    #endif

    // Start I2C communication
    Wire1.begin();
    delay( 250 );

    // Read - Unit ID "Who Am I"
    sensorID = readSensorID(); // Read the sensor ID
    #ifdef JDEBUG
    Serial.printf( " INFO: MPU6050 Init ID: %#x, ADO: %d, I2C Address: %#x\n"
                   , sensorID, sensorADO, addressI2C  
                );
    #endif

    if( MPU6050_SUCCESS != setPowerState( POWER_WAKE ) )
    { // Set the power state to wake
        Serial.printf( "ERROR: MPU6050 Init: failed to set power state [WAKE]\n" );
    }
    else
    {
        Serial.printf( " INFO: MPU6050 Init: readPowerRegister()=%2#x [%s]\n"
                       , powerReg
                       , POWER_STRING(powerState)
                     );
    }

    if( MPU6050_SUCCESS != setTempState( TEMPERATURE_ENABLE ) )
    { // Set the temperature state to enable
        Serial.printf( "ERROR: MPU6050 Init: failed to set temperature state [ENABLE]\n" );
    }
    else
    {
        #ifdef JDEBUG
        // Initial read of the temperature
        Serial.printf( " INFO: MPU6050 Init: readTempeRegister()=(%02#x:%02#x)\n"
                    , tempRaw[0], tempRaw[1]
                    );
        Serial.printf( " INFO: MPU6050 Init: temperature: %f [%sed]\n"
                       , temperature, TEMPERATURE_STRING(temperatureState) );
        #endif
    }


    // Set Gyro Sensitivity
    #define GYRO_FS_SEL         0x01 // 0=250, 1=500, 2=1000, 3=2000  scale in: +/- [degrees/sec]
    #define GYRO_CONFIG         0x1B // Gyro Configuration Register 
    #define GYRO_CONFIG_SIZE       1 // Gyro Configuration Register size 

    GyroSensitivity /= pow( 2, GYRO_FS_SEL );
    if( MPU6050_SUCCESS != readI2cRegisters( (uint8_t)GYRO_CONFIG, (uint8_t *)&GyroSensitivityFS, (uint8_t)GYRO_CONFIG_SIZE ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 Init: failed to read gyro sensitivity\n" );
        GyroSensitivity = GYRO_SENSITIVITY;
        GyroSensitivityFS = 0;
    }
    else if( GyroSensitivityFS |= (GYRO_FS_SEL << 3),  // Set to +/- 16g
             MPU6050_SUCCESS != writeI2cRegisters( (uint8_t)GYRO_CONFIG, (uint8_t *)&GyroSensitivityFS, (uint8_t)GYRO_CONFIG_SIZE )
           )
    { // Write failed
        Serial.printf( "ERROR: MPU6050 Init: failed to set gyro sensitivity\n" );
        GyroSensitivity = GYRO_SENSITIVITY;
        GyroSensitivityFS = 0;
    }
    else
    { // Write was successful
        Serial.printf( " INFO: MPU6050 Init: Gyro Sensitivity: %f, FS_SEL: %#x, REG: %02#x\n"
                       , GyroSensitivity, GYRO_FS_SEL, GyroSensitivityFS
                     );
    }


    // Set Accel Sensitivity
    #define ACCEL_FS_SEL         0x02 // 0=2g, 1=4g, 2=8g, 3=16g  range: +/- [ g ]
    #define ACCEL_CONFIG         0x1C // Accelerometer Configuration Register 
    #define ACCEL_CONFIG_SIZE       1 // Accelerometer Configuration Register size

    AccelSensitivity /= pow( 2, ACCEL_FS_SEL );
    if( MPU6050_SUCCESS != readI2cRegisters( (uint8_t)ACCEL_CONFIG, (uint8_t *)&AccelSensitivityFS, (uint8_t)ACCEL_CONFIG_SIZE ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 Init: failed to read accelerometer sensitivity\n" );
        AccelSensitivity = ACCEL_SENSITIVITY;
        AccelSensitivityFS = 0;
    }
    else if( AccelSensitivityFS |= (ACCEL_FS_SEL << 3),
             MPU6050_SUCCESS != writeI2cRegisters( (uint8_t)ACCEL_CONFIG, (uint8_t *)&AccelSensitivityFS, (uint8_t)ACCEL_CONFIG_SIZE )
           )
    { // Write failed
        Serial.printf( "ERROR: MPU6050 Init: failed to set accelerometer sensitivity\n" );
        AccelSensitivity = ACCEL_SENSITIVITY;
        AccelSensitivityFS = 0;
    }
    else
    { // Write was successful
        Serial.printf( " INFO: MPU6050 Init: Accel Sensitivity: %f, FS_SEL: %#x, REG: %02#x\n"
                    , AccelSensitivity, ACCEL_FS_SEL, AccelSensitivityFS
                    );
    }

    // Low Pass Filter
    #define ACCEL_DLPF_CFG      0x05    // 0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
    #define ACCEL_CONFIG2       0x1D    // Low Pass Filter Configuration Register
    #define ACCEL_CONFIG2_SIZE     1    // Low Pass Filter Configuration Register size
    uint8_t AccelDLPF = ACCEL_DLPF_CFG; // Set to 10Hz
    if( MPU6050_SUCCESS != readI2cRegisters( (uint8_t)ACCEL_CONFIG2, (uint8_t *)&AccelDLPF, (uint8_t)ACCEL_CONFIG2_SIZE ) )
    {
        Serial.printf( "ERROR: MPU6050 Init: failed to read accelerometer low pass filter\n" );
        AccelDLPF = 0;
    }
    else if( AccelDLPF |= (ACCEL_DLPF_CFG << 3),
             MPU6050_SUCCESS != writeI2cRegisters( (uint8_t)ACCEL_CONFIG2, (uint8_t *)&AccelDLPF, (uint8_t)ACCEL_CONFIG2_SIZE )
           )
    { // Write failed
        Serial.printf( "ERROR: MPU6050 Init: failed to set accelerometer low pass filter\n" );
        AccelDLPF = 0;
    }
    else
    { // Write was successful
        Serial.printf( " INFO: MPU6050 Init: Accel Low Pass Filter: %d, REG: %02#x\n"
                       , AccelDLPF, ACCEL_DLPF_CFG
                     );
    }


    gyroCalibrate( );
    accelCalibrate( );

}

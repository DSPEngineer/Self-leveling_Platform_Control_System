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
#define  GYRO_REG_SIZE      6
#define  ACCEL_REG_BASE     0x3B
#define  ACCEL_REG_SIZE      6

#define  PWR_RESET_BIT      0x80
#define  PWR_SLEEP_BIT      0x40
#define  TEMP_DISABLE_BIT   0x08
#define  PWR_CLK_SEL_MASK   0x07

#define SERIAL_BAUD_RATE  115200

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
    ERR_CODE retCode = MPU6050_ERROR;
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
    else if( 0 == i2cReadSize )
    { // No data was read
        Serial.printf( "ERROR: MPU6050 readI2cRegisters: no data read\n" );
        retCode = MPU6050_I2C_READ;
    }
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
        Serial.printf( " WARN: MPU6050 setTempState() -- invalid temperature state (%d)\n"
                       ,newState
                    );
        retVal = MPU6050_INVALID_TEMP_STATE;
    }
/*
    else if( state == temperatureState )
    { // The current state matches the proposed (new) state
        Serial.printf( " WARN: MPU6050 setTempState() -- temperature state unchanged, (%s)\n"
                       , TEMPERATURE_STRING( state )
                    );
    }
*/
    else if( MPU6050_SUCCESS != ( retVal = readPowerRegister() ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 setTempState() failed to read power register with error [%d]\n", retVal );
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
            Serial.printf( "ERROR: failed to select the MPU6050 power register\n" );
            retVal = MPU6050_I2C_WRITE; // Set to invalid value
        }
        else if( 1 != Wire1.write( powerReg ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to set temperature state on the MPU6050\n" );
            retVal = MPU6050_I2C_WRITE; // Set to invalid value
        }
        else
        {  // Write successfully sent temperature sensor offset
            Serial.printf( " INFO: MPU6050 temperature state set to %s\n", TEMPERATURE_STRING( temperatureState ) );
            retVal = MPU6050_SUCCESS;
        }

        // Always try to end the transmission
        int cmdRet = Wire1.endTransmission();

        if( 0 != cmdRet )
        { // Report if ending transmission fails
            Serial.printf( "ERROR: MPU6050 sensorReset failed with error [%d]\n", retVal );
            retVal = MPU6050_I2C_ERROR; // Set to invalid value
        }
/*        else if ( MPU6050_SUCCESS != ( retVal = readTemperature() ) )
        { // Read failed
            Serial.printf( "ERROR: MPU6050 getTemperature() failed to read current value, error [%d]\n", retVal );
        }
*/
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
        //#ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 getTemperature -- Temp: %f\n", *temp );
        //#endif
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
//            temperature = (float)( (tempRaw[1] << 8 ) | ( tempRaw[0] & 0x00FF ) );
//            temperature = (float)( ( ( tempRaw & 0x00FF ) << 8 ) |  ( (tempRaw & 0xFF00) >> 8) );
            temperature = (int16_t)( (readData[0] << 8) | readData[1] );
            //#ifdef DEBUG
            Serial.printf( " INFO: MPU6050 readTemperatureRegister -- read registers (%02#x:%02#x), Temp: [%f] \n"
                           , readData[0], readData[1], temperature
                         );
 
            //#endif
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
        *x = (GyroX / 131.0 );
        *y = (GyroY / 131.0 );
        *z = (GyroZ / 131.0 );
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 getGyroValues -- GyroX: %04#x, GyroY: %04#x, GyroZ: %04#x\n"
                       , (uint16_t)*x, (uint16_t)*y, (uint16_t)*z
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
    { // Read was successful
        GyroX = ( (gyroRaw[0] >> 8) ) | ( (gyroRaw[0] & 0xFF) << 8 );
        GyroY = ( (gyroRaw[1] >> 8) ) | ( (gyroRaw[1] & 0xFF) << 8 );
        GyroZ = ( (gyroRaw[2] >> 8) ) | ( (gyroRaw[2] & 0xFF) << 8 );
        
        #ifdef JDEBUG
        Serial.printf( " INFO: MPU6050 readGyroRegisters -- GyroX: %04#x, GyroY: %04#x, GyroZ: %04#x\n"
                       , (uint16_t)GyroX, (uint16_t)GyroY, (uint16_t)GyroZ
                     );
        #endif
    }
    return retVal;

}


/***********************************************************************
**
************************************************************************/
mpu6050::ERR_CODE
//mpu6050::getAccelValues( uint16_t *x, uint16_t *y, uint16_t *z )
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
        *x = ( AccelX / 16384.0);
        *y = ( AccelY / 16384.0);
        *z = ( AccelZ / 16384.0);
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
    { // Read was successful
        AccelX =  ( (accelRaw[0] >> 8) ) | ( (accelRaw[0] & 0x00FF) << 8 );
        AccelY =  ( (accelRaw[1] >> 8) ) | ( (accelRaw[1] & 0x00FF) << 8 );
        AccelZ =  ( (accelRaw[2] >> 8) ) | ( (accelRaw[2] & 0x00FF) << 8 );

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
        }

    }

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


    // Setting the Filter
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x1A); // Starting with register 0x1A (Low Pass FIlter)
    // Wire1.write(0x05); // Set the low pass filter to 10Hz
    // Wire1.endTransmission();
  
    // // Sensitivity scale factor for Gyroscope:
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x1B); // Starting with register 0x1B (Sensitivity scale factor)
    // Wire1.write(0x08); // Set the scale factor bits [3:4], value=1 ( +/- 500 deg/sec )
    // Wire1.endTransmission();
  
}

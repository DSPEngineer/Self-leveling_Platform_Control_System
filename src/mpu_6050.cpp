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

#define  PWR_RESET_BIT      0x80
#define  PWR_SLEEP_BIT      0x40
#define  TEMP_DISABLE_BIT   0x08


/***********************************************************************
**  Constructor
************************************************************************/
mpu6050::mpu6050( uint8_t ado )
{
    Serial.printf( "INFO: MPU6050 Constructor\n" );

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
    powerState = readPowerRegister(); // Initialize power register

    // Initialize the device
    init();

    sensorID = readSensorID(); // Read the sensor ID
    Serial.printf( "INFO: MPU6050 ID: %#x, ADO: %d, I2C Address: %#x\n"
                   , sensorID, sensorADO, addressI2C  
                );
}


/***********************************************************************
** Destructor
************************************************************************/
mpu6050::~mpu6050()
{
    // Destructor
    Serial.printf( "INFO: MPU6050 Destructor\n" );
    Wire1.end(); // Stop I2C communication

}


/***********************************************************************
** Public - return current power state
************************************************************************/
mpu6050::POWER_STATE
mpu6050::getPowerState()
{
    return ( ( powerState & PWR_SLEEP_BIT ) ? POWER_SLEEP : POWER_WAKE );
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
    int i2cErr = 0;
    uint8_t i2cReadSize = 0;

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
        Serial.printf( "ERROR: MPU6050 read size is too large (%d)\n", size );
        retCode = MPU6050_INVALID_SIZE;
    }
    else if( Wire1.beginTransmission( addressI2C ), // MPU6050 I2C address                
             Wire1.write( reg ),                    // register offset
             ( MPU6050_SUCCESS != ( i2cErr = Wire1.endTransmission() ) )
           )
    {
        Serial.printf( "ERROR: MPU6050 set read address failed with error (%d)\n", i2cErr );
        retCode = MPU6050_I2C_WRITE;
    }
    else if(  0 == ( i2cReadSize = Wire1.requestFrom( addressI2C, size ) ) )
    { // Request the data from the MPU6050
        Serial.printf( "ERROR: MPU6050 request read failed with error (%d)\n", i2cReadSize );
        retCode = MPU6050_I2C_ERROR;
    }
    else if( i2cReadSize != size )
    { // Requested size is wrong
        Serial.printf( "ERROR: MPU6050 read size mismatch (%d != %d)\n", i2cReadSize, size );
        retCode = MPU6050_I2C_READ;
    }
    else if( 0 == i2cReadSize )
    { // No data was read
        Serial.printf( "ERROR: MPU6050 no data read\n" );
        retCode = MPU6050_I2C_READ;
    }
    else
    { // Read the I2C registers

        for( uint8_t i = 0; i < i2cReadSize; i++ )
        {
            data[i] = Wire1.read();
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
    int i2cErr = 0;

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
    { // Logic to write the registers
        Wire1.beginTransmission( addressI2C ); // MPU6050 start with I2C address

        if( 1 != Wire1.write( reg ) )          // write register offset
        {
            Serial.printf( "ERROR: MPU6050 set write address failed with error (%d)\n", i2cErr );
            retCode = MPU6050_I2C_WRITE;
        }
        else if( Wire1.write( data, size ) != size )
        { // Writing data failed
            Serial.printf( "ERROR: MPU6050 write register(s) failed with error (%d)\n", i2cErr );
            retCode = MPU6050_I2C_WRITE;
        }
        else if( 0 != Wire1.endTransmission() )
        { // Check final result
            Serial.printf( "ERROR: MPU6050 write register(s) failed with error (%d)\n", i2cErr );
            retCode = MPU6050_I2C_ERROR;
        }
        else
        {
            retCode = MPU6050_SUCCESS;
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
        Serial.printf( "ERROR: MPU6050 readSensorID() failed with error [%d]\n", errCode );
        sensorID = 0xFF; // Set to invalid value
    }
    else
    { // Read was successful
        Serial.printf( "INFO: MPU6050 readSensorID() -- ID: %#x\n", sensorID );
    }

    return sensorID;
}






/***********************************************************************
**
************************************************************************/
// Public - reset the power state
mpu6050::POWER_STATE
mpu6050::setPowerState(POWER_STATE newState)
{
    uint8_t currentPowerReg = 0xFF;
    ERR_CODE retCode = MPU6050_ERROR;

    Serial.printf( "WARN: MPU6050 setPowerState() -- new power state: (%d) [%s]\n"
                   ,newState
                   ,POWER_STRING( newState )
                 );

    Serial.printf( "WARN: MPU6050 setPowerState() -- current power state: (%d) [%s]\n"
                   ,powerState
                   ,POWER_STRING( powerState )
                 );

    if( newState == powerState)
    { // The current state matches the proposed (new) state
        Serial.printf( "WARN: MPU6050 setPowerState() -- device state unchanged, (%s)\n"
                       ,POWER_STRING( newState )
                    );
    }
    else if( MPU6050_SUCCESS != ( retCode = readI2cRegisters( (uint8_t)PWR_MGMT_1, (uint8_t *)&currentPowerReg, (uint8_t)1 ) ) )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 setPowerState() failed to read power register with error [%d]\n", retCode );
        powerState = POWER_FAILED; // Set to invalid value
    }
    else
    {
        if( POWER_SLEEP == newState )
        { // Set the new power value and send to MPU6050
            currentPowerReg |= PWR_SLEEP_BIT;
            powerState = POWER_SLEEP;
        }
        else
        { 
            currentPowerReg &= ~PWR_SLEEP_BIT;
            powerState = POWER_WAKE;
        }

        Wire1.beginTransmission( addressI2C ); // MPU6050 I2C address
        // Point to Power management register
        if( 1 != Wire1.write( PWR_MGMT_1 ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to select the MPU6050 power register\n" );
            powerState = POWER_FAILED; // Set to invalid value
        }
        else if( 1 != Wire1.write( currentPowerReg ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to set power state on the MPU6050\n" );
            powerState = POWER_FAILED; // Set to invalid value
        }
        else
        {  // Write successfully sent a single byte
            Serial.printf( "INFO: MPU6050 power state set to %s\n", POWER_STRING( powerState ) );
        }
        Wire1.endTransmission();
    }

    return powerState;
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


/***********************************************************************
** Private:
**   read the power register and compute POWER STATE and
**   write result to powerReg
************************************************************************/
mpu6050::POWER_STATE
mpu6050::
readPowerRegister( void )
{ // read the value of the power register 
    uint8_t powerReg = 0xFF;

    ERR_CODE errRet = readI2cRegisters( (uint8_t)PWR_MGMT_1, (uint8_t *)&powerReg, (uint8_t)1 );

    if( MPU6050_SUCCESS != errRet )
    { // Read failed
        Serial.printf( "ERROR: MPU6050 readPowerRegister() failed with error [%d]\n", errRet );
        powerState = POWER_FAILED; // Set to invalid value
    }
    else
    { // Read was successful
        Serial.printf( "INFO: MPU6050 readPowerRegister() -- ID: %#x\n", powerReg );
        powerState = ( powerReg & PWR_SLEEP_BIT ) ? POWER_SLEEP : POWER_WAKE;
    }

    Serial.printf( "INFO: MPU6050 readPowerRegister() -- Power State: [%s]\n"
                   ,POWER_STRING( powerState )
                 );

    return powerState;
}


/***********************************************************************
**
************************************************************************/
void
mpu6050::
sensorReset( void )
{
    u_char currentPowerState = readPowerRegister();

    Wire1.beginTransmission( addressI2C ); // MPU6050 I2C address
    Wire1.write( PWR_MGMT_1 );
    Wire1.write( currentPowerState | PWR_RESET_BIT );
    Wire1.endTransmission();
}


/***********************************************************************
**
************************************************************************/
void 
mpu6050::init( void )
{
    Wire1.setClock( I2C_CLOCK_SPEED ); // Set I2C clock speed to 400kHz
    Wire1.begin(); // Start I2C communication
    delay( 250 );

    // Set the power register value, in this class
    int regVal = readPowerRegister();
    Serial.printf( "INFO: MPU6050 Init: readPowerRegister()=%#x (%s)\n", regVal
                   ,( POWER_WAKE == regVal ) ? "WAKE" : "SLEEP"
                 );

    // // Filter
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x1A); // Starting with register 0x1A (Low Pass FIlter)
    // Wire1.write(0x05); // Set the low pass filter to 10Hz
    // Wire1.endTransmission();
  
    // // Sensitivity scale factor for Gyroscope:
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x1B); // Starting with register 0x1B (Sensitivity scale factor)
    // Wire1.write(0x08); // Set the scale factor bits [3:4], value=1 ( +/- 500 deg/sec )
    // Wire1.endTransmission();
  
    // // Unit ID "Who Am I"
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x75); // Who Am I register
    // Wire1.endTransmission();
  
}

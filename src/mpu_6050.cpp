///////////////////////////////////////////////////////////////////////////////
// Source file for MPU 6050 class
//  Used to read and write data.
//
//  Jose Pagan
//  04/10/2025
///////////////////////////////////////////////////////////////////////////////

#include <mpu_6050.h>

#define  PWR_MGMT_1     0x6B
#define  PWR_MGMT_2     0x6C

#define  PWR_RESET_BIT  0x80
#define  PWR_SLEEP_BIT  0x40

u_char
mpu6050::getSensorID(void)
{
    // Unit ID "Who Am I"                                                         
    Wire1.beginTransmission( sensorAddress ); // MPU6050 I2C address                
    Wire1.write(0x75); // Who Am I register                                       
    Wire1.endTransmission();

    Wire1.requestFrom( sensorAddress, 1); // Request 1 byte from the MPU6050        
//    while( !Wire1.available() );
    sensorID = Wire1.read();

    return sensorID;
}


// Constructor
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

    sensorAddress = I2C_BASE + sensorADO; // Compute the I2C address    
    powerReg = 0xff; // Initialize power register

    // Initialize the device
    init();

    sensorID = getSensorID(); // Read the sensor ID
    Serial.printf( "INFO: MPU6050 ID: %#x, ADO: %d, Address: %#x\n"
                   , sensorID, sensorADO, sensorAddress  
                );
}


// Destructor
mpu6050::~mpu6050()
{
    // Destructor
    Serial.printf( "INFO: MPU6050 Destructor\n" );
}


// Public - return current power state
mpu6050::POWER_STATE
mpu6050::getPowerState(void)
{
    return ( (powerReg & 0x40) ? POWER_OFF : POWER_ON );
}


// Public - reset the power state
mpu6050::POWER_STATE
mpu6050::setPowerState(POWER_STATE newState)
{
    POWER_STATE previousState = getPowerState();

    Serial.printf( "INFO: setPowerState() -- current state: %#x (%s)\n", powerReg
        , (POWER_ON == powerReg) ? "WAKE" : "SLEEP"
        );
    Serial.printf( "INFO: setPowerState() --     new state: %#x (%s)\n", newState
            , (POWER_ON == newState) ? "WAKE" : "SLEEP"
            );
    
    if( newState == previousState )
    { // The current state matches the proposed (new) state
        Serial.printf( "WARN: MPU6050 device state unchanged, (%s)\n"
                    , (POWER_ON == newState) ? "POWER_ON" : "POWER_OFF"
                    );
    }
    else
    {
        writePowerRegister( newState );
    }

    return previousState;
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// Private - read the power register and write to powerReg  
mpu6050::POWER_STATE
mpu6050::readPowerRegister( void )
{ // read the value of the power register 
    POWER_STATE retState = POWER_FAILED;

    Wire1.beginTransmission( sensorAddress ); // MPU6050 I2C address

    if( 1 != Wire1.write( PWR_MGMT_1 ) )
    { // the write failed to send one byte
        Serial.printf( "ERROR: failed to select the MPU6050 power register\n" );
    }
    else
    { // Read next char from the MPU6050
        Wire1.requestFrom( sensorAddress, 1 ); // Request 1 byte from the MPU6050
        while(  !Wire1.available() );
        // { // the read failed to return one byte
        //     Serial.printf( "ERROR: failed to read the MPU6050 power register\n" );
        // }
        // else
        // { // Read successfully returned a single byte
        //     Serial.printf( "INFO: MPU6050 power register read\n" );
        // }
        powerReg = Wire1.read();
        Serial.printf( " INFO: Read Power Register: %#x\n", powerReg);
        retState = ( powerReg & PWR_SLEEP_BIT ) ? POWER_OFF : POWER_ON;
        Serial.printf( " INFO: Register State: %#x (%s)\n", retState, (POWER_OFF == retState) ?"SLEEP":"WAKE" );
    }

    Wire1.endTransmission();

    return retState;
}


// Private - write the power register and write value to powerReg  
mpu6050::POWER_STATE
mpu6050::writePowerRegister( mpu6050::POWER_STATE newState)
{
    POWER_STATE ret = POWER_FAILED;
    // Enable MPU 6050 - use Power register 0x6B
    //    Bit6 -> sleep

    POWER_STATE curState = (powerReg & PWR_SLEEP_BIT) ? POWER_ON : POWER_OFF;
    Serial.printf( "WARN: MPU6050 device state (%#x)\n", curState );

    Serial.printf( " --> Cur: %#x, New: %#x\n", curState, newState );
    if ( newState == curState )
    {
        ret = curState;
        Serial.printf( "WARN: MPU6050 device state unchanged, (%s)\n"
                       ,(curState == POWER_ON) ? "WAKE" : "SLEEP"
                    );
    }
    else
    {
        // Set the new power value and send to MPU6050
        u_char newPwrValue = powerReg;

        if ( POWER_ON == newState )
        {
            newPwrValue &= PWR_SLEEP_BIT;
        }
        else
        { 
            newPwrValue |= PWR_SLEEP_BIT;
        }

        Serial.printf( " --> Cur: %#x, New: %#x\n", powerReg, newPwrValue );

        Wire1.beginTransmission( I2C_BASE + sensorAddress ); // MPU6050 I2C address

        // Point to Power management register
        if( 1 != Wire1.write( PWR_MGMT_1 ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to select the MPU6050 power register\n" );
        }
        else if( 1 != Wire1.write( newPwrValue ) )
        { // the write failed to send one byte
            Serial.printf( "ERROR: failed to set power state on the MPU6050\n" );
        }
        else
        {  // Write successfully sent a single byte
            ret = (newPwrValue & PWR_SLEEP_BIT) ? POWER_OFF : POWER_ON;
            Serial.printf( "INFO: MPU6050 power state set to %s\n"
                           ,( POWER_ON == ret ) ? "POWER_ON" : "POWER_OFF" );
        }

        Wire1.endTransmission();
    }

    return ret;
}



void mpu6050::init( void )
{
    Wire1.setClock( I2C_CLOCK_SPEED ); // Set I2C clock speed to 400kHz
    Wire1.begin(); // Start I2C communication
    delay( 250 );

    // Set the power register value, in this class
    int regVal = readPowerRegister();
    Serial.printf( "INFO: MPU6050 Init: readPowerRegister()=%#x (%s)\n", regVal
        ,( POWER_ON == regVal ) ? "WAKE" : "SLEEP" );

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
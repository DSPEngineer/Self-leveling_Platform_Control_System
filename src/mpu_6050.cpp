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

#define  PWR_SLEEP_BIT  0x40

mpu6050::mpu6050()
{
    init();
}


mpu6050::POWER_STATE
mpu6050::readPowerRegister( void )
{ // read the value of the power register 
    POWER_STATE retState = POWER_FAILED;

    Wire1.beginTransmission( I2C_BASE + sensorID ); // MPU6050 I2C address

    if( 1 != Wire1.write( PWR_MGMT_1 ) )
    { // the write failed to send one byte
        Serial.printf( "ERROR: failed to select the MPU6050 power register\n" );
    }
    else
    { // Read next char from the MPU6050
        powerReg = Wire1.read();
        retState = ( powerReg & PWR_SLEEP_BIT ) ? POWER_ON : POWER_OFF;
    }

    Wire1.endTransmission();

    return retState;
}


mpu6050::POWER_STATE
mpu6050::setPowerRegister( mpu6050::POWER_STATE newState)
{
    POWER_STATE ret = POWER_FAILED;
    // Enable MPU 6050 - use Power register 0x6B
    //    Bit6 -> sleep

    POWER_STATE curState = (powerReg & PWR_SLEEP_BIT) ? POWER_ON : POWER_OFF;

    if ( newState == curState )
    {
        ret = curState;
        Serial.printf( "WARN: MPU6050 device state unchanged, (%s)\n"
            , (curState == POWER_ON) ? "POWER_ON" : "POWER_OFF"
            );
    }
    else
    {
        // Set the new power value and send to MPU6050
        u_char newPwrValue = powerReg;

        if ( POWER_ON == newState )
        {
            newPwrValue |= PWR_SLEEP_BIT;
        }
        else
        { 
            newPwrValue &= ~PWR_SLEEP_BIT;
        }

        Wire1.beginTransmission( I2C_BASE + sensorID ); // MPU6050 I2C address

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
            ret = (newPwrValue & PWR_SLEEP_BIT) ? POWER_ON : POWER_OFF;
            Serial.printf( "INFO: MPU6050 power state set to %s\n"
                           ,( POWER_ON == ret ) ? "POWER_ON" : "POWER_OFF" );
        }

        Wire1.endTransmission();
    }

    return ret;
}


mpu6050::POWER_STATE
mpu6050::getPowerState(void)
{
    return ( (powerReg & 0x40) ? POWER_ON : POWER_OFF );
}


mpu6050::POWER_STATE
mpu6050::setPowerState(POWER_STATE newState)
{
    POWER_STATE previousState = getPowerState();

    if( newState == previousState )
    { // The current state matches the proposed (new) state
        Serial.printf( "WARN: MPU6050 device state unchanged, (%s)\n"
                    , (POWER_ON == newState) ? "POWER_ON" : "POWER_OFF"
                    );
    }
    else
    {
        setPowerRegister( newState );
    }

    return previousState;
}

void mpu6050::init( void )
{
    Wire1.setClock( I2C_CLOCK_SPEED ); // Set I2C clock speed to 400kHz
    Wire1.begin(); // Start I2C communication
    // delay( 250 );

    // Set the power register value, in this class
    readPowerRegister();

    // // Enable MPU 6050 - use Power register
    // Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
    // Wire1.write(0x6B); // Gyro-Base Register 43 X 6
    // Wire1.write(0x00); // Set the power management register to 0
    // Wire1.endTransmission();
  
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
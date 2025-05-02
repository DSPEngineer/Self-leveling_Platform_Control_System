///////////////////////////////////////////////////////////////////////////////
// Source file for MG996R servo class
//  Used to init, move servo motor using PWM
//
//  Jose Pagan
//  04/21/2025
///////////////////////////////////////////////////////////////////////////////

#include <servoLib.h>

// Constructor
servoLib::
servoLib( uint8_t pin, float frequency )
{
    Serial.printf( "INFO: servoLib Constructor\n" );
    init( pin, frequency );
}


// Destructor
servoLib::
~servoLib()
{
    Serial.printf( "INFO: servoLib Destructor\n" );
}


// Common initialization
void 
servoLib::
init( uint8_t pin, float frequency )
{
    Serial.printf( "INFO: MG996R_Servo init() - Pin: %d, Frequency: %d\n", pin, frequency );

    // Set the PWM frequency
    analogWriteFrequency( pin, frequency ); 
    analogWrite(pin, 128 );
}


// Common initialization
uint16_t 
servoLib::
setAngle( uint16_t angle )
{
    u_int16_t retVal = 0xFFFF;        // Return an error

    if( 180 < angle )
    { // Angle is above max range
        Serial.printf( "ERROR: servoAngle - angle out of range (%d)\n", angle );
        retVal = MAX_COUNT;
    }
    else if( 0 == angle)
    { // Angle is min
        retVal = MIN_COUNT;
    }
    else if( 90 == angle)
    { // Angle is middle
        retVal = MID_COUNT;
    }
    else
    { // Angle is in range
        retVal = MIN_COUNT + ( ( angle * ( MAX_COUNT - MIN_COUNT ) ) / 180 );
    }

    analogWrite( this->pin, retVal ); 

    return retVal;
}
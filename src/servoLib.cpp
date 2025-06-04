///////////////////////////////////////////////////////////////////////////////
// Source file for MG996R servo class
//  Used to init, move servo motor using PWM
//
//  Jose Pagan
//  04/21/2025
///////////////////////////////////////////////////////////////////////////////

#include <servoLib.h>


// // Constructor
// servoLib::
// servoLib( uint8_t pin, uint frequency ) : pin(pin), frequency( frequency) 
// {
//     Serial.printf( " INFO: MG996R servoLib Constructor\n" );
//     init( );
// }

// Constructor
servoLib::servoLib( uint8_t pin,
                    uint freq,
                    uint16_t minAng,
                    uint16_t maxAng
                  ) : pin(pin), frequency( freq ) , minServoAngle( minAng ), maxServoAngle( maxAng )
{
    Serial.printf( " INFO: MG996R servoLib Constructor\n" );
    init( );
}


// Destructor
servoLib::
~servoLib()
{
    Serial.printf( " INFO: MG996R servoLib Destructor\n" );
}


// Common initialization
void 
servoLib::
init( void )
{
    Serial.printf( " INFO: MG996R Servo init() - Pin: %d, Frequency: %d\n", pin, frequency );

    // Set the PWM frequency
    analogWriteFrequency( pin, frequency ); 

    // for( int angle = MAX_ANGLE; angle > MIN_ANGLE; angle -= 45 )
    // { // Set the servo to the initial position
    //     setAngle( angle );
    //     delay(1000);
    // }

    // Start at neutral angle
    // setAngle( MID_ANGLE );
    // setAngle( 0 );
    delay(2000);
}


// Common initialization
uint16_t
servoLib::
setAngle( uint16_t angle )
{
    uint16_t retVal = angle;        // Return an error

    if( maxServoAngle < angle )
    { // Angle is above max range
        Serial.printf( "ERROR: MG996R servoLib setAngle - pin (%d) angle greater than MAX range (%d)\n", pin, angle );
        //retVal = maxServoAngle;
    }
    else if( minServoAngle > angle )
    { // Angle is min
        Serial.printf( "ERROR: MG996R servoLib setAngle - pin (%d) angle less than MIN range (%d)\n", pin, angle );
        //retVal = minServoAngle;
    }
    else
    { // Angle is in range
        retVal = angle;
    }
    #ifdef JDEBUG
    Serial.printf( " INFO: MG996R servoLib setAngle - angle %d, val %d\n", angle,retVal );
    #endif
    analogWrite( this->pin, retVal ); 

    return retVal;
}
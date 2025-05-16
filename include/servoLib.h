///////////////////////////////////////////////////////////////////////////////
// Header file for MG996R servo class
//  Used to init, move servo motor using PWM
//
//  Jose Pagan
//  04/21/2025
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <Arduino.h>
#include <Wire.h>

#define PWM_FREQUENCY    50 // 50 Hz - default for MG996R Servo motor
//#define PWM_FREQUENCY    100 // 100 Hz
//#define PWM_FREQUENCY    200 // 200 Hz
//#define PWM_FREQUENCY    300 // 300 Hz
//#define PWM_FREQUENCY    400 // 400 Hz
//#define PWM_FREQUENCY    500 // 500 Hz
//#define PWM_FREQUENCY    600 // 600 Hz

#define  WAVELENGTH  ceil( 1000.0  / PWM_FREQUENCY )  // in [ ms ]
#define  INCREMENT   floor( 256.0 / WAVELENGTH ) // in  [ val /m s ]
#define  MIN_COUNT   floor(  INCREMENT / 1.9 )
#define  MAX_COUNT   floor( MIN_COUNT + ( 2.4 * INCREMENT ) )
#define   MID_COUNT   ( MIN_COUNT + (MAX_COUNT - MIN_COUNT ) / 2.0 )

#define MAX_ANGLE  270
#define MIN_ANGLE  50
#define MID_ANGLE  ( (MAX_ANGLE + MIN_ANGLE) / 2 )

class servoLib
{
    public:
        servoLib( uint8_t pin, uint frequency = PWM_FREQUENCY );
        ~servoLib();

        uint16_t setAngle( uint16_t angle );
      
    private:
        uint8_t pin;
        uint    frequency;

//        void init( u_int8_t pin, float frequency );
        void init( void );

};

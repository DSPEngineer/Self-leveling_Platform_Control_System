///////////////////////////////////////////////////////////////////////////////
// Header file for Blinking LED's
//  Used to init, flash LED's using PWM
//
//  Jose Pagan
//  04/21/2025
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <Arduino.h>
#include <Wire.h>

#define PWM_FREQUENCY    50 // 50 Hz - default for MG996R Servo motor
#define PWM_DUTY_CYCLE   50 // Default duty cycle for LED's

class ledLib
{
    public:
        ledLib( uint8_t pin,
                  uint freq = PWM_FREQUENCY,
                  uint16_t dutyCycle = PWM_DUTY_CYCLE
                );

        ~ledLib();

      
    private:
        uint8_t     pin;
        uint        frequency;
        uint16_t    dutyCycle;

        void init( void );

};
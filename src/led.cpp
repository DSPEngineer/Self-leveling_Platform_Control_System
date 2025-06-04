///////////////////////////////////////////////////////////////////////////////
// Source file for Blinking LED's
//  Used to init, flash LED's using PWM
//
//  Jose Pagan
//  04/21/2025
///////////////////////////////////////////////////////////////////////////////

#include <led.h>

ledLib::ledLib(
    uint8_t pin,
    uint freq ,
    uint16_t dutyCycle
   ) : pin(pin), frequency( freq ), dutyCycle( dutyCycle )
{
    
}


ledLib::~ledLib()
{

}

void
ledLib::init()
{
    Serial.printf( " INFO: LED init() - Pin: %d, Frequency: %d, Duty Cycle: %d\n", pin, frequency, dutyCycle );

    // Set the PWM frequency
    analogWriteFrequency( pin, frequency ); 

    // Set the PWM duty cycle
    analogWrite( pin, dutyCycle );
    
    delay(2000);
}

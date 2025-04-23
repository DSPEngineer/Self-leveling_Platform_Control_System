#include <Arduino.h>
//#include <accelerometer_utils.h>



void setup()
{

  Serial.begin(115200);   
  Serial.print( "SETUP: BEE-526 - Self-leveling Platform Control System\n" );
  Serial.print( "BEE-526 - Self-leveling Platform Control System\n" );
//  Serial.printf( "Result: %d\n", result ); // This should print 5
  
  Serial.print( " DONE: BEE-526 - Self-leveling Platform Control System\n" );    
}

int count=0;
void loop()
{  // put your main code here, to run repeatedly:
  Serial1.print( "LOOP: BEE-526 - \n" );
  Serial1.printf( "LOOP: BEE-526 - Self-leveling Platform Control System - %d\n", count++ );
}

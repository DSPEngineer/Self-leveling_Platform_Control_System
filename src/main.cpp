#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup()
{
  int result = myFunction(2, 3);
  Serial.begin(115200);   
  Serial.print( "SETUP: BEE-526 - Self-leveling Platform Control System\n" );
  Serial.print( "BEE-526 - Self-leveling Platform Control System\n" );
  Serial.printf( "Result: %d\n", result ); // This should print 5
  
  Serial.print( " DONE: BEE-526 - Self-leveling Platform Control System\n" );    
}

void loop()
{  // put your main code here, to run repeatedly:
  Serial1.print( "LOOP: BEE-526 - Self-leveling Platform Control System\n" );
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
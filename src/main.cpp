#include <Arduino.h>
#include <mpu_6050.h>
//#include <accelerometer_utils.h>

//int mySerial = Serial.begin(115200);

#define SERIAL_BAUD_RATE  115200
//#define PWM_FREQUENCY    75000000 // 75 MHz
//#define PWM_FREQUENCY    300 // 300 Hz
#define PWM_FREQUENCY    50 // 50 Hz

mpu6050 mySensor;

void setup()
{
  delay(1000);

  Serial.begin( SERIAL_BAUD_RATE );
  delay(2000);

  Serial.printf( " INFO: Serial Baud: %d\n", SERIAL_BAUD_RATE );
  delay(250);

  Wire1.begin(); // Start I2C communication
  delay(1000);

  
  Serial.print( "SETUP: BEE-526 - Self-leveling Platform Control System\n" );
  Serial.print( "   BEE-526 - Self-leveling Platform Control System\n" );
  // Serial.printf( "Result: %d\n", result ); // This should print 5
  Serial.printf( "    * Current state: %s \n"
               , (mpu6050::POWER_ON == mySensor.getPowerState()) ? "WAKE" : "SLEEP"
             );

  mySensor.setPowerState( mpu6050::POWER_OFF );
  mySensor.setPowerState( mpu6050::POWER_ON );

  analogWriteFrequency(0, PWM_FREQUENCY ); 
  analogWrite(0, 128 );

  Serial.print( "SDONE: BEE-526 - Self-leveling Platform Control System\n" );
}


// # define  MAX_COUNT  ( ( PWM_FREQUENCY * 66  ) / 100 )
// # define  MIN_COUNT  ( ( (PWM_FREQUENCY / 50) * 66 ) / 100 )
// #define   MID_COUNT  ( MIN_COUNT + ( ( MAX_COUNT - MIN_COUNT )  / 2 ) )

# define  WAVELENGTH  ceil( 1000.0  / PWM_FREQUENCY )  // in [ ms ]
# define  INCREMENT   floor( 256.0 / WAVELENGTH ) // in  [ val /m s ]
# define  MIN_COUNT   floor(  INCREMENT / 1.9 )
# define  MAX_COUNT   floor( MIN_COUNT + ( 2.4 * INCREMENT ) )

#define   MID_COUNT   ( MIN_COUNT + (MAX_COUNT - MIN_COUNT ) / 2.0 )

int count = MID_COUNT;
// Min count = 5
// Max count = 32

void loop()
{
  analogWrite(0, count );
  Serial.printf( "LOOP: BEE-526 - %d, Pwr: %#x \n", count, mySensor.getPowerState() );
//    Serial1.printf( "LOOP: BEE-526 - Self-leveling Platform Control System - %d\n", count++ );
  delay(125);

  if (count > MAX_COUNT)
  {
    count = MIN_COUNT;
    analogWrite(0, count );
    delay(1000);
  }
  else
    count += 1; //INCREMENT;    

//  analogWrite(0, 0 );
//  delay(5000);

}

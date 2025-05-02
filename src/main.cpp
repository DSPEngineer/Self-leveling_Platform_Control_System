#include <Arduino.h>
#include <mpu_6050.h>
#include <servoLib.h>
//#include <accelerometer_utils.h>

//int mySerial = Serial.begin(115200);

#define SERIAL_BAUD_RATE  115200
//#define PWM_FREQUENCY    75000000 // 75 MHz
//#define PWM_FREQUENCY    300 // 300 Hz
//#define PWM_FREQUENCY    50 // 50 Hz

mpu6050   mySensor;
servoLib  servo(0);

void setup()
{
  delay(1000);

  Serial.begin( SERIAL_BAUD_RATE );
  delay(8000);

  Serial.print( "SETUP: BEE-526 - Self-leveling Platform Control System\n" );
  Serial.printf( " INFO: Serial Baud: %d\n", SERIAL_BAUD_RATE );
  delay(250);

  Wire1.begin(); // Start I2C communication
  delay(1000);
  
  Serial.printf( "    * Sensor ID: %#x \n",  mySensor.getSensorID() );

//  Serial.printf( "    * Power Reg: %#x\n",  mySensor.readPowerRegister() );
  Serial.printf( "    * Current state: %s\n"
    , (mpu6050::POWER_WAKE == mySensor.getPowerState()) ? "WAKE" : "SLEEP"
   );

   mySensor.setPowerState( mpu6050::POWER_WAKE );
   mySensor.setPowerState( mpu6050::POWER_SLEEP );
  mySensor.setPowerState( mpu6050::POWER_WAKE );

  servo.setAngle( 90 );

  Serial.print( "SDONE: BEE-526 - Self-leveling Platform Control System\n" );
}


uint16_t motorAngle = 90;

void loop()
{
  servo.setAngle( motorAngle );
  Serial.printf( "LOOP: BEE-526 - %d, Pwr: %#x \n", motorAngle, mySensor.getPowerState() );
  delay(125);

  if ( motorAngle > 120) 
  {
    motorAngle = 60; // MIN_COUNT;
    analogWrite(0, motorAngle );
    delay(10);
  }
  else if ( motorAngle < 60) 
  {
    motorAngle = 60; // MIN_COUNT;
    analogWrite(0, motorAngle );
    delay(10);
  }
  else
    motorAngle += 1; //INCREMENT;    

}

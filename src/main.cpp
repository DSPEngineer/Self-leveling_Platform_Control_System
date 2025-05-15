#include <Arduino.h>
#include <mpu_6050.h>
#include <servoLib.h>
#include <IntervalTimer.h>
//#include <accelerometer_utils.h>

#define SERIAL_BAUD_RATE  115200

unsigned long prevTime = 0;
unsigned long currTime = 0;

mpu6050   *mySensor = NULL;
servoLib  servo(0);

void setup()
{
  delay(2000);

  Serial.begin( SERIAL_BAUD_RATE );
  //delay(3000);

  Serial.print( "SETUP: BEE-526 - Self-leveling Platform Control System\n" );
  Serial.printf( " INFO: Serial Baud: %d\n", SERIAL_BAUD_RATE );
  delay(250);

  mySensor = new mpu6050( 0 ); // ADO = 0
  mySensor->sensorReset();
  delay(1000);

  mySensor->setPowerState( mpu6050::POWER_WAKE );
 // delay(1000);

  Serial.printf( "    * Sensor ID: %#x \n",  mySensor->getSensorID() );

//  Serial.printf( "    * Power Reg: %#x\n",  mySensor->readPowerRegister() );
  Serial.printf(
    "    * Current state: %d [%s]\n"
    , mySensor->getPowerState()
    , POWER_STRING( mySensor->getPowerState() )
   );

  mySensor->setPowerState( mpu6050::POWER_WAKE );
  mySensor->setPowerState( mpu6050::POWER_SLEEP );
  mySensor->setPowerState( mpu6050::POWER_WAKE );

  mySensor->setTempState( mpu6050::TEMPERATURE_ENABLE );

  servo.setAngle( 90 );

  Serial.print( "SDONE: BEE-526 - Self-leveling Platform Control System\n" );
  prevTime = millis();
  delay(100);
}


uint16_t motorAngle = 90;

 float accAngleX  = 0;
 float accAngleY  = 0;
 float gyroAngleX = 0;
 float gyroAngleY = 0;
 float gyroAngleZ = 0;

float roll  = 0;
float pitch = 0;
float yaw   = 0;

void loop()
{
  // Temp Values
  float temp = -1;

  currTime = millis();
  unsigned long msecTime = ( currTime - prevTime );   // in [ ms ]
  float secTime = msecTime  / 1000.0;                 // in [ sec ]

  // // // // // // // // // // // // // // // // // // // // // // //
  // Temperature
  mpu6050::ERR_CODE err = mySensor->getTemperature( &temp );

  if( mpu6050::MPU6050_SUCCESS != err )
  { // Read failed
    Serial.printf( "ERROR: MPU6050 getTemperature failed with error [%d]\n", err );
    temp = -1;
  }
  else
  {
    servo.setAngle( motorAngle );
    #ifdef JDEBUG
    Serial.printf( "LOOP: BEE-526 - Angle: %d [deg], Pwr: [%s], Temp: [%f C, %f F] \n"
                   , motorAngle
                   , POWER_STRING(mySensor->getPowerState())
                   , ( (float)temp / 340 ) + 36.53
                   , ( ( (float)temp / 340 ) + 36.53 ) * 9.0 / 5.0 + 32
                 );
#endif
//    delay(125);
  }

  // // // // // // // // // // // // // // // // // // // // // // //
  // Gyro Values
  float gx, gy, gz;
  err = mySensor->getGyroValues( &gx, &gy, &gz );
  if( mpu6050::MPU6050_SUCCESS != err )
  { // Read failed
    Serial.printf( "ERROR: MPU6050 getGyroValues failed with error [%d]\n", err );
    temp = -1;
  }
  else
  {  // Correct the outputs with the calculated error values
    // gx += 0.56; // GyroErrorX ~(-0.56)
    // gy -= 2.00; // GyroErrorY ~(2)
    // gz += 0.79; // GyroErrorZ ~ (-0.8)
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX += gx * secTime; // deg/s * s = deg
    gyroAngleY += gy * secTime;
    yaw +=  gz * secTime;
  }


  // // // // // // // // // // // // // // // // // // // // // // //
  // Accel Values
  float ax, ay, az;
  //int16_t ax, ay, az;
  err = mySensor->getAccelValues( &ax, &ay, &az );
  prevTime = millis();

  if( mpu6050::MPU6050_SUCCESS != err )
  { // Read failed
    Serial.printf( "ERROR: MPU6050 getGyroValues failed with error [%d]\n", err );
  }
  else
  {
    accAngleX = ( atan(      ay / sqrt( pow(ax, 2) + pow(az, 2) ) ) * 180 / PI ) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = ( atan( -1 * ax / sqrt( pow(ay, 2) + pow(az, 2) ) ) * 180 / PI ) + 1.58; // AccErrorY ~(-1.58)
//    delay(125);
  }

  // Complementary filter - combine accelerometer and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  servo.setAngle( motorAngle );
  // Serial.printf( "LOOP: BEE-526 - GyroX: %04#x,GyroY: %04#x,GyroZ: %04#x  \n"
  //                 , (int16_t)gx, (int16_t)gy, (uint16_t)gz
  //              );
  Serial.printf( "LOOP: BEE-526 - Time: (%u [ms]) %f [s], Temp=%f"
                  , msecTime, secTime, temp
                );
  Serial.printf( "  |  <DIR> Roll=%f   Pitch=%f   Yaw=%f"
                  , roll, pitch, yaw
                );
  Serial.printf( "  |  <Gyr> X=%f  Y:=%f  Z=%f   |  <Acc> X=%f  Y=%f  Z=%f  |  <AccAngle> X=%f  Y=%f"
                  , gx, gy, gz, ax, ay, az, accAngleX, accAngleY
                );
  Serial.printf( "\n" );

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

  prevTime = millis();
  delay(500);

}

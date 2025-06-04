///////////////////////////////////////////////////////////////////////////////
// Code for accelerometer functions for the Teensy 4.1
//  Used to read data for the MPU 6050
//
//  Jose Pagan
//  04/10/2025
///////////////////////////////////////////////////////////////////////////////
//#pragma GCC optimize ("O0")

#include<accelerometer_utils.h>

// const uint16_t I2C_CHANNEL = 0x68;
// uint16_t sensorID = 0;

// float RateRoll  = 0;
// float RatePitch = 0;
// float RateYaw   = 0;

// void gyroSignals(void)
// {
//   // // Filter
//   // Wire.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   // Wire.write(0x1A); // Starting with register 0x1A (Low Pass FIlter)
//   // Wire.write(0x05); // Set the low pass filter to 10Hz
//   // Wire.endTransmission();

//   // // Sensitivity scale factor for Gyroscope:
//   // Wire.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   // Wire.write(0x1B); // Starting with register 0x1B (Sensitivity scale factor)
//   // Wire.write(0x08); // Set the scale factor bits [3:4], value=1 ( +/- 500 deg/sec )
//   // Wire.endTransmission();

//   // Access Gyro Measurements:
//   Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   Wire1.write(0x43); // Gyro-Base Register begins at 0x43 ( X 6 readings)
//   Wire1.endTransmission();
  
//   // Begin reading from the setting above:
//   Wire1.requestFrom( I2C_CHANNEL, 6); // Request 6 bytes from the MPU6050

//   {
//     int8_t dataHi = 0;
//     int8_t dataLo = 0;

//     // Wait for data to be available
//     dataHi = Wire1.read();
//     dataLo = Wire1.read();
// //    Serial.printf( "\t Roll:  Hi: [%#x]   Lo: [%#x]", dataHi, dataLo);
//     RateRoll  = (float) ( (uint16_t)( dataHi << 8 | dataLo ) ) /65.5;

//     dataHi = Wire1.read();
//     dataLo = Wire1.read();
// //    Serial.printf( "\tPitch:  Hi: [%#x]   Lo: [%#x]", dataHi, dataLo);
//     RatePitch = (float) ( (uint16_t)( dataHi << 8 | dataLo ) ) /65.5;

//     dataHi = Wire1.read();
//     dataLo = Wire1.read();
// //    Serial.printf( "\t  Yaw:  Hi: [%#x]   Lo: [%#x]", dataHi, dataLo);
//     RateYaw   = (float) ( (uint16_t)( dataHi << 8 | dataLo ) ) /65.5;
//   }
// }

// void setup()
// {
//   int result = myFunction(2, 3);
//   Serial.begin( 115200);

//   Serial.printf( "%#8x : ID (%#x) : Starting: %d\n\n", I2C_CHANNEL, sensorID, result );

//   pinMode( LED_BUILTIN, OUTPUT); // Initialize the built-in LED pin as an output

//   Wire1.setClock( 400000 ); // Set I2C clock speed to 400kHz
//   Wire1.begin(); // Start I2C communication
//   delay( 250 );

//   // Enable MPU 6050 - use Power register
//   Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   Wire1.write(0x6B); // Gyro-Base Register 43 X 6
//   Wire1.write(0x00); // Set the power management register to 0
//   Wire1.endTransmission();

//   // Filter
//   Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   Wire1.write(0x1A); // Starting with register 0x1A (Low Pass FIlter)
//   Wire1.write(0x05); // Set the low pass filter to 10Hz
//   Wire1.endTransmission();

//   // Sensitivity scale factor for Gyroscope:
//   Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   Wire1.write(0x1B); // Starting with register 0x1B (Sensitivity scale factor)
//   Wire1.write(0x08); // Set the scale factor bits [3:4], value=1 ( +/- 500 deg/sec )
//   Wire1.endTransmission();

//   // Unit ID "Who Am I"
//   Wire1.beginTransmission( I2C_CHANNEL ); // MPU6050 I2C address
//   Wire1.write(0x75); // Who Am I register
//   Wire1.endTransmission();
  
//   Wire1.requestFrom( I2C_CHANNEL, 1); // Request 1 byte from the MPU6050
//   while( Wire1.available() )
//     sensorID = Wire1.read();
  
// }

// int loopCount = 0;
// void loop()
// {
//   Serial.printf( "%#2x : ID: %#8x : Loop Count: %d\t --> ", I2C_CHANNEL, sensorID, loopCount++ );
//   digitalWrite( LED_BUILTIN, HIGH); // Turn the LED on (HIGH is the voltage level)
//   delay( 100 );

//   // gyroSignals();
//   // Serial.printf( "   Roll: %8.4f\t\t", RateRoll );
//   // Serial.printf( "\tPitch: %8.4f\t", RatePitch );
//   // Serial.printf( "\t  Yaw: %8.4f\n", RateYaw );


//   digitalWrite( LED_BUILTIN, LOW ); // Turn the LED on (HIGH is the voltage level)
//   delay( 100 );

//   gyroSignals();
//   Serial.printf( "   Roll: %f\t\t", RateRoll );
//   Serial.printf( "\tPitch: %f\t", RatePitch );
//   Serial.printf( "\t  Yaw: %f\n", RateYaw );
//   //Serial.printf( "\n" );

// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }

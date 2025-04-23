///////////////////////////////////////////////////////////////////////////////
// Header file for MPU 6050 class
//  Used to read and write data.
//
//  Jose Pagan
//  04/10/2025
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <Arduino.h>
//#include <MPU6050.h>
#include <Wire.h>


class mpu6050
{
    public:
        //u_int8_t enum { POWER_OFF, POWER_ON } POWER_STATE;
        enum POWER_STATE { POWER_FAILED=-1, POWER_OFF, POWER_ON };

        mpu6050();

        // Chip power state methods 
        // Accessor - returns current state
        POWER_STATE getPowerState(void);
        // Mutator - set new state and return previous state
        POWER_STATE setPowerState(POWER_STATE state);

        //        void gyroSignals(void);
//        void setFilter(uint8_t filter);
//        void setSensitivity(uint8_t sensitivity);
//        uint16_t getSensorID(void);
//        float getRateRoll(void);
//        float getRatePitch(void);
//        float getRateYaw(void);

    private:
        void init(void);
        POWER_STATE setPowerRegister( POWER_STATE newState);
        POWER_STATE readPowerRegister(void);

        const uint32_t  I2C_CLOCK_SPEED     = 400000;   // MPU6050 I2C address
        const uint8_t   I2C_BASE            = 0x68;     // MPU6050 I2C address
        uint8_t         sensorID            = 0;        // Should reflect ADO

        u_char          powerReg            = 0x40;     // Power management register


        float RateRoll                      = 0;
        float RatePitch                     = 0;
        float RateYaw                       = 0;
        

};


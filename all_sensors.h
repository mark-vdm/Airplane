/*
*   This is the main header file
*   Contains all includes
*/

#ifndef ALL_SENSORS_H_INCLUDED
#define ALL_SENSORS_H_INCLUDED

#include <Arduino.h>


#include "MPU6050_6Axis_MotionApps20.h"
#include "MemoryFree.h"
#include "I2Cdev.h"
#include "Airplane.h"
//#include "DMP.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int update_imu();   //call this to update IMU values (must call often)
int initialize_imu(); //call this to start the IMU (call once)
extern Airplane a;

extern MPU6050 mpu;
extern volatile bool mpuInterrupt;
extern Airplane a;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
extern bool dmpReady;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#endif // ALL_SENSORS_H_INCLUDED

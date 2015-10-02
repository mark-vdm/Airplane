/*
*   This is the main header file
*   Contains all includes
*/

#ifndef ALL_SENSORS_H_INCLUDED
#define ALL_SENSORS_H_INCLUDED

#include <Arduino.h>


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MemoryFree.h"
#include "I2Cdev.h"
#include "Airplane.h"
#include "NewPing.h"
//#include "DMP.h"


//// SERVOS AND RECEIVER //////////////////////////////////////
// Flags to indicate when a servo position should be updated
//extern volatile uint8_t ServoUpdateFlags;



//Pin change interrupts
#include <PinChangeInt.h>
#include "RCArduinoFastLib.h"
volatile uint8_t bUpdateFlagsShared;


// SERVO VALUES
#define THROTTLE_IN_PIN 3 //The pins for each servo control
#define ROLL_IN_PIN 4
#define PITCH_IN_PIN 5
#define YAW_IN_PIN 6
#define MODE_IN_PIN 7

// Assign your channel out pins
#define THROTTLE_OUT_PIN 8
#define AIL_L_OUT_PIN A0
#define AIL_R_OUT_PIN A1
#define RUDDER_OUT_PIN A2
#define ELEVATOR_OUT_PIN A3


#define THROTTLE_ID 0 //The index of 'servos' for each servo control
#define AIL_R_ID   1
#define AIL_L_ID   2
#define RUDDER_ID  3
#define ELEVATOR_ID 4
#define SERVO_FRAME_SPACE 5


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define ROLL_FLAG 2
#define PITCH_FLAG 4
#define YAW_FLAG 8
#define MODE_FLAG 16

// The flags to indicate when the servos get new signals are in AIRPLANE.H

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
//volatile uint16_t unSteeringInShared;
//volatile uint16_t unAuxInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unThrottleInStart;
//uint16_t unSteeringInStart;
//uint16_t unAuxInStart;

//uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

void calcThrottle();
//void calcSteering();
//void calcAux();
void update_receiver(); //updates values from receiver
void update_servos();   //updates output to servo


/////////////////////////////////////////////////////////////////////////////

// ULTRASONIC SENSORS //
#define TRIGGER_PIN 9 //arduino pin on trigger
#define ECHO_PIN 9 //Arduino pin to echo
#define TRIGGER_PINR 10 //arduino pin on trigger
#define ECHO_PINR 10 //Arduino pin to echo
#define MAX_DISTANCE 300 //Maximum distance for ping (cm)

///////////////////////////////////////////////////////////////////

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int update_imu();   //call this to update IMU values (must call often)
int initialize_imu(); //call this to start the IMU (call once)
int initialize_SD();  //call this to start the SD card (call once)
extern Airplane a;

extern MPU6050 mpu;
extern volatile bool mpuInterrupt;
//extern Airplane a;

//Ultrasonic vars
NewPing ultra_bot(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing ultra_rear(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
uint8_t ULTRA_SELECT = 0;

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
VectorInt16 gy;
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#endif // ALL_SENSORS_H_INCLUDED

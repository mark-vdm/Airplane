#ifndef CLASS_AIRPLANE_H_INCLUDED
#define CLASS_AIRPLANE_H_INCLUDED

//#define SD_CARD_USED //Uncomment to use SD card functions (very old - might be broken)
#include <Arduino.h>
#include "helper_3dmath.h"
#include "MemoryFree.h"
#include "Servo.h"
//#include "SoftwareServo.h"

// SERVO VALUES
#define THROTTLE_PIN 1 //The pins for each servo control
#define AIL_R_PIN A1
#define AIL_L_PIN A0
#define RUDDER_PIN A2
#define ELEVATOR_PIN A3
#define SERVO_MAX_PULSE_RATE 50 //the time between servo pulses (ms)

#define THROTTLE_ID 0 //The index of 'servos' for each servo control
#define AIL_R_ID   1
#define AIL_L_ID   2
#define RUDDER_ID  3
#define ELEVATOR_ID 4

// RECEIVER FUNCTIONS //
//#include <PinChangeInt.h> - Cannot include this here. Compiler fails


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define ROLL_FLAG 2
#define PITCH_FLAG 4
#define YAW_FLAG 8
#define MODE_FLAG 16



// holds the update flags defined above
//volatile uint8_t bUpdateFlagsShared;


#ifdef SD_CARD_USED
#include <SPI.h>
#include <SD.h>
//#include <SdFat.h>
//extern SdFat SD;
//#include <string.h>
const uint8_t chipSelect = 10;
#endif



// MARK'S PROGRAM VARIABLES AND STUFF
//ARM_FLAG: indicates when the motor+servos are armed. Signalled from radio controller
//bool aARM_RAD_FLAG;  // TRUE when radio sends true signal (!-100)
//bool aARM_ARD_FLAG; // Set to false when ARM_FLAG is false. Set true when arduino receives arm cmd

struct sensordata{
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 gy;     // [x, y, z]            gravity-free accel sensor measurements
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    uint8_t ult_b;
    uint8_t ult_r;
    unsigned long dt;
};

class Airplane{
    public:
        Airplane(); //constructor
        void outpt();

        sensordata dat; //holds onto the raw sensor values
        void print_sensors(uint8_t select);
 int control();
        //Servos
        //SoftwareServo servos[5]; // throttle, ail_l, ail_r, rud, elevator
        //Servo servos[5]; // throttle, ail_l, ail_r, rud, elevator

        int servoPos[5];
        //unsigned long servoTime[5]; //save the last time servo was set (ms)

//        void servo_set(); //makes sure to only update the servo every 10ms

        uint16_t flight_index; //flight number - used for data file name(0-9999)
        uint8_t log_index; //recorded subsection of flight -(0-255)
    private:

};



#endif // CLASS_AIRPLANE_H_INCLUDED

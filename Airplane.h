#ifndef CLASS_AIRPLANE_H_INCLUDED
#define CLASS_AIRPLANE_H_INCLUDED

//#define SD_CARD_USED //Uncomment to use SD card functions (very old - might be broken)
#include <Arduino.h>
#include "helper_3dmath.h"
#include "MemoryFree.h"

// Flags to indicate when a servo position should be updated
#define SERVO_FLAG_THROTTLE 1
#define SERVO_FLAG_R 2
#define SERVO_FLAG_L 4
#define SERVO_FLAG_RUD 8
#define SERVO_FLAG_ELEV 16
extern volatile uint8_t ServoUpdateFlags; //extern b/c it is used in the main file

#define THROTTLE_ID 5 //The index of 'servos' for each servo control
#define AIL_R_ID   0
#define AIL_L_ID   1
#define RUDDER_ID  2
#define ELEVATOR_ID 3
#define SERVO_FRAME_SPACE 4
#define SERVO_FRAME_SPACE2 6

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
struct receiver{ //stores the values from the receiver
    int throttle;
    int yaw;
    int pitch;
    int roll;
    int mode;
};

class Airplane{
    public:
        Airplane(); //constructor
        void outpt();

        sensordata dat; //holds onto the raw sensor values
        receiver rc;    //holds onto the raw receiver values
        void print_sensors(uint8_t select);
 int control();
        //Servos
        //SoftwareServo servos[5]; // throttle, ail_l, ail_r, rud, elevator
        //Servo servos[5]; // throttle, ail_l, ail_r, rud, elevator

        int servoPos[6];
        //unsigned long servoTime[5]; //save the last time servo was set (ms)

//        void servo_set(); //makes sure to only update the servo every 10ms

        int servo_offset[6]; //offsets for servos
        uint8_t servo_scaling[6]; //scaling for servo (percentage)

        uint16_t flight_index; //flight number - used for data file name(0-9999)
        uint8_t log_index; //recorded subsection of flight -(0-255)
    private:
        //Routines for different flight modes
        void mode_stop();
        void mode_airplane();
        void mode_heli1();
        void mode_heli2();
        void add_offset(); //add the offsets for each servo
};



#endif // CLASS_AIRPLANE_H_INCLUDED

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

//Flight Modes
#define error_max 5  //the maximum error in the value 'mode' allowed
#define MODE_STOP -100
#define MODE_FLIGHT 0
#define MODE_HELI   10
#define MODE_HELI2  20

#define BATT_PIN A6


//Constants
#define SIN45 0.707106

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
    int batt; //voltage of battery (millivolts)
};
struct receiver{ //stores the values from the receiver
    int throttle;
    int yaw;
    int pitch;
    int roll;
    int mode;
};
struct state{ //stores the control system states
    Quaternion angle;       //current angle
    Quaternion angle_desire;   //desired angle
    Quaternion angle_off_q; // This quaternion points from the AIRPLANE orientation to the DESIRED orientation. angle*angle_off_q = angle_desired
    VectorFloat test_angle; //FOR TEST USE ONLY
    //VectorFloat angle_off;
    VectorFloat angle_v; //Angle of airplane, vector form. Useful for finding inclination and yaw. Z-component is inclination
    //VectorFloat angle_d; //SLERP between actual angle and desired angle
    //VectorFloat Iangle_off; //integral of angle offset (for integral controller)

    //Vectors for the x,y, and z-axis offsets
    VectorFloat x_vect;
    VectorFloat y_vect;
    VectorFloat z_vect;


    VectorInt16 pos_g;     //global position [mm]
    VectorFloat v_g;       //global velocity
    VectorFloat a_g;        //global accel
    VectorFloat v_p;        //airplane velocity
    VectorFloat a_p;        //airplane accel
    VectorFloat F;          //airplane forces
    VectorFloat M;          //airplane moment
    VectorFloat vAng;      //airplane angular velocity (omega)
    VectorFloat aAng;      //airplane angular accel (alpha)
    int16_t flaps;          //gets the current position of each servo

    //float incline;  //This is the inclination of the airplane (angle from horizontal) (horizontal = 0, vertical = 90). Equal to angle_v.z
};

class Airplane{
    public:
        Airplane(); //constructor
        void outpt();

        state X; //holds the control state of the airplane
        sensordata dat; //holds onto the raw sensor values
        receiver rc;    //holds onto the raw receiver values
        void print_sensors(uint8_t select);
 int control();
 void update_state(); //updates the state prediction
 void desired_angle(); //calculates the desired angle
        //Servos
        //SoftwareServo servos[5]; // throttle, ail_l, ail_r, rud, elevator
        //Servo servos[5]; // throttle, ail_l, ail_r, rud, elevator

        int servoPos[6];
        //unsigned long servoTime[5]; //save the last time servo was set (ms)

//        void servo_set(); //makes sure to only update the servo every 10ms

        int servo_offset[6]; //offsets for servos
        int servo_scaling[6]; //scaling for servo (percentage)

        uint16_t flight_index; //flight number - used for data file name(0-9999)
        uint8_t log_index; //recorded subsection of flight -(0-255)
        int check_batt();
    private:
        //Routines for different flight modes
        void mode_stop();
        void mode_airplane(); //direct control of ctrl surfaces
        void mode_heli1();    //tries to fly vertical
        void mode_heli2();
        void add_offset(); //add the offsets for each servo
        int limit(int value, int deg); //limit the range of servo
};



#endif // CLASS_AIRPLANE_H_INCLUDED

#ifndef CLASS_AIRPLANE_H_INCLUDED
#define CLASS_AIRPLANE_H_INCLUDED

//#define SD_CARD_USED //Uncomment to use SD card functions (very old - might be broken)
#include <Arduino.h>
#include "helper_3dmath.h"
#include "MemoryFree.h"

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
/*
        //Ultrasonic Functions/*
        NewPing ultra_bot();
        NewPing ultra_rear();
        unsigned long pingTimer; //holds the next ping time
        void ultraB_ping();
        void ultraR_ping();
        friend void echoCheck();*/
        //send ping
        //check distance from ISR

        // SD CARD FUNCTIONS: DISABLED //
        #ifdef SD_CARD_USED
        File myFile;
        //int init_SD(); //This does not work. Only works when in the main file
        int index_log(); //get the index of the previous log FIN
        int SD_newLog(); //create new log file
        void SD_close(); //close current log file
        #endif

        uint16_t flight_index; //flight number - used for data file name(0-9999)
        uint8_t log_index; //recorded subsection of flight -(0-255)
    private:

};



#endif // CLASS_AIRPLANE_H_INCLUDED

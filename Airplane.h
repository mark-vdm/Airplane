#ifndef CLASS_AIRPLANE_H_INCLUDED
#define CLASS_AIRPLANE_H_INCLUDED

//#define SD_CARD_USED //Uncomment to use SD card functions (very old - might be broken)
#include <Arduino.h>

#ifdef SD_CARD_USED
#include <SPI.h>
#include <SD.h>
//#include <SdFat.h>
//extern SdFat SD;
#include <string.h>
const uint8_t chipSelect = 10;
#endif

//#include "NewPing.h"

// ULTRASONIC SENSORS //
//#define TRIGGER_PIN 8 //arduino pin on trigger
//#define ECHO_PIN 8 //Arduino pin to echo
//#define TRIGGER_PINR 9 //arduino pin on trigger
//#define ECHO_PINR 9 //Arduino pin to echo
//#define MAX_DISTANCE 300 //Maximum distance for ping (cm)

//extern MPU6050 mpu;

// MARK'S PROGRAM VARIABLES AND STUFF
//ARM_FLAG: indicates when the motor+servos are armed. Signalled from radio controller
//bool aARM_RAD_FLAG;  // TRUE when radio sends true signal (!-100)
//bool aARM_ARD_FLAG; // Set to false when ARM_FLAG is false. Set true when arduino receives arm cmd


class Airplane{
    public:
        Airplane(); //constructor
        void outpt();

        //sensordata dat;

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

struct sensordata{
};

#endif // CLASS_AIRPLANE_H_INCLUDED

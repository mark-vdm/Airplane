#ifndef CLASS_AIRPLANE_H_INCLUDED
#define CLASS_AIRPLANE_H_INCLUDED

//#include "MPU6050_6Axis_MotionApps20.h"
//#include "MemoryFree.h"
//#include "DMP.h"
#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;

//extern MPU6050 mpu;

// MARK'S PROGRAM VARIABLES AND STUFF
//ARM_FLAG: indicates when the motor+servos are armed. Signalled from radio controller
//bool aARM_RAD_FLAG;  // TRUE when radio sends true signal (!-100)
//bool aARM_ARD_FLAG; // Set to false when ARM_FLAG is false. Set true when arduino receives arm cmd

class Airplane{
    public:
        Airplane(); //constructor
        void outpt();
//        bool create_file(); //Adds another data file. Increments title. Adds unique ID for the set of runs
        File myFile;

    private:
        uint16_t flight_id; //flight number - used for data file name
        uint8_t set_id; //recorded subsection of flight - used for data file name
};

struct sensordata {

};

#endif // CLASS_AIRPLANE_H_INCLUDED

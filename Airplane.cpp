#include "Airplane.h"

//MPU6050 mpu;

Airplane::Airplane(){
    //Default constructor
//    blinkState = false;
    // MPU control/status vars
  //  dmpReady = false;

  log_index = 0;
  flight_index = 0;
    sensordata dat = {}; //initialize all data values to zero
    receiver rc = {1500};
    rc.throttle = 1000;

    ServoUpdateFlags = 1+2+4+8+16; //start the UpdateFlags as 1
    for (int i = 0; i++; i<5){
        servoPos[i] = 1500;
    }
  //NewPing ultra_bot(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);// = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  //NewPing ultra_rear(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);// = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  //pingTimer = millis(); //start timer for ultrasonics
}

void Airplane::outpt(){
//    Serial.println("I'm here");
}

int Airplane::control(){
    //This is the main control algorithm for the airplane. Call every cycle
    int servoPosPrev[5];

    //copy the previous servo values
    for (uint8_t i = 0; i++; i<5)
        servoPosPrev[i] = servoPos[i];

    if (abs(servoPos[THROTTLE_ID] - rc.throttle) >10)
        servoPos[THROTTLE_ID] = rc.throttle; //send the throttle value directly to output
    servoPos[RUDDER_ID] = rc.yaw;



    // if any servo value was updated, set the update flag to 1
    if (servoPosPrev[THROTTLE_ID] != servoPos[THROTTLE_ID])
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_THROTTLE;
    if (servoPosPrev[RUDDER_ID] != servoPos[RUDDER_ID])
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_RUD;
}

/*void Airplane::servo_set(){
    //make sure more than 20ms has passed since last servo command
    for(int i = 0; i++; i<5){
        //if ((millis() > (servoTime[i] + SERVO_MAX_PULSE_RATE))  //check if 20ms
        //     && servos[i].readMicroseconds() != servoPos[i])    //check if there was a change
        //{
            //servos[i].writeMicroseconds(servoPos[5]);
            servos[i].write(servoPos[5]);
            servoTime[i] = millis();
        //}
    }
}*/



void Airplane::print_sensors(uint8_t select){
    //Print the selected values of sensors - indicated by bit status
    //0 - IMU ypr       bin: 1 hex: 01
    //1 - IMU acc       bin: 10 hex: 02
    //2 - IMU gyr       bin: 100 hex: 04
    //3 - Ultrasonic    bin: 1000 hex: 08
    //4 - Memory useage bin: 10000 hex: 10
    //5 - RC receiver   bin: 100000  hex: 20
    //6    hex: 40
    //7    hex: 80
    if(select){ //if any options are selected...
        if (select & 0x01){ // IMU ypr
            Serial.print("ypr\t");
            Serial.print(dat.ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[2] * 180/M_PI);
            Serial.print("\t");
        }
        if (select & 0x02){
            Serial.print("acc\t");
            Serial.print(dat.aa.x);
            Serial.print("\t");
            Serial.print(dat.aa.y);
            Serial.print("\t");
            Serial.print(dat.aa.z);
            Serial.print("\t");
        }
       if (select & 0x04){
            Serial.print("gy\t");
            Serial.print(dat.gy.x);
            Serial.print("\t");
            Serial.print(dat.gy.y);
            Serial.print("\t");
            Serial.print(dat.gy.z);
            Serial.print("\t");
        }
        if (select & 0x08){
            Serial.print("UltraB: ");
            Serial.print(dat.ult_b);
            Serial.print("\t UltraR: ");
            Serial.print(dat.ult_r);
        }
        if (select & 0x10){
            Serial.print("\t cpu: ");
            Serial.print(freeMemory());
        }
        if (select & 0x20){
            Serial.print("\t radio thr: ");
            Serial.print(rc.throttle);
            Serial.print("\t y:");
            Serial.print(rc.yaw);
        }
        Serial.println("");
    }
}

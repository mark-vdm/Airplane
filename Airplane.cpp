#include "Airplane.h"

//MPU6050 mpu;

Airplane::Airplane(){
    //Default constructor

    sensordata dat = {}; //initialize all data values to zero
    receiver rc = {1500};
    rc.throttle = 1000;

    //set the servo offsets for each servo
    servo_offset[AIL_R_ID]=-190-300; //this is always changed to a random number for no reason
    servo_offset[AIL_L_ID]=-340;
    servo_offset[RUDDER_ID]=-150 - 30;
    servo_offset[ELEVATOR_ID]=-80 + 230; //offset to zero servo, + offset to make elevator zero
    servo_offset[THROTTLE_ID]=0;

    servo_scaling[AIL_R_ID]=100;
    servo_scaling[AIL_L_ID]=100;
    servo_scaling[RUDDER_ID]=100;
    servo_scaling[ELEVATOR_ID]=100;
    servo_scaling[THROTTLE_ID]=100;

    ServoUpdateFlags = 1+2+4+8+16; //start the UpdateFlags as 1
    for (int i = 0; i++; i<6){
        servoPos[i] = 1500;
    }
}

void Airplane::mode_stop(){
//If the left toggle (SF) is LOW, cut power and stuff

}

void Airplane::outpt(){
//    Serial.println("I'm here");
}
void Airplane::add_offset(){
    for (int i = 0; i<6; i++)
        servoPos[i] += servo_offset[i]; //send the throttle value directly to output
}

int Airplane::control(){
    //This is the main control algorithm for the airplane. Call every cycle
    int servoPosPrev[5];


    //copy the previous servo values
    for (uint8_t i = 0; i++; i<5)
        servoPosPrev[i] = servoPos[i];

    //Set new positions for the servos
    //if (abs(servoPos[THROTTLE_ID] - rc.throttle) >10)
    servoPos[THROTTLE_ID] = rc.throttle; //send the throttle value directly to output
    servoPos[RUDDER_ID] = rc.yaw;
    servoPos[ELEVATOR_ID] = rc.pitch;
    servoPos[AIL_L_ID] = rc.roll;
    servoPos[AIL_R_ID] = rc.roll;




    add_offset(); //add offsets to each servo position

//servo_offset[AIL_R_ID]=10;
    // if any servo value was updated, set the update flag to 1
    if (servoPosPrev[THROTTLE_ID] - servoPos[THROTTLE_ID])
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_THROTTLE;
    if (abs(servoPosPrev[RUDDER_ID] - servoPos[RUDDER_ID])>10)
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_RUD;
    if (abs(servoPosPrev[ELEVATOR_ID] - servoPos[ELEVATOR_ID])>10)
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_ELEV;
    if (abs(servoPosPrev[AIL_L_ID] - servoPos[AIL_L_ID])>10)
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_L;
    if (abs(servoPosPrev[AIL_R_ID] - servoPos[AIL_R_ID])>10)
        ServoUpdateFlags = ServoUpdateFlags | SERVO_FLAG_R;

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
            Serial.print("\t p:");
            Serial.print(rc.pitch);
            Serial.print("\t r:");
            Serial.print(rc.roll);
            Serial.print("\t m:");
            Serial.print(rc.mode);
        }
        if (select & 0x40){
            Serial.print("\t Servo Thr: ");
            Serial.print(servoPos[THROTTLE_ID]);
            Serial.print("\t Ail L:");
            Serial.print(servoPos[AIL_L_ID]);
            Serial.print("\t Ail R:");
            Serial.print(servoPos[AIL_R_ID]);
            Serial.print("\t Elev:");
            Serial.print(servoPos[ELEVATOR_ID]);
            Serial.print("\t Rud:");
            Serial.print(servoPos[RUDDER_ID]);

        }
        Serial.println("");
    }
}

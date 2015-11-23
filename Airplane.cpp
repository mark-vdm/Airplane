#include "Airplane.h"

//MPU6050 mpu;

Airplane::Airplane(){
    //Default constructor

    sensordata dat = {}; //initialize all data values to zero
    receiver rc = {1500};
    rc.throttle = 1000;

    //set the servo offsets for each servo
    servo_offset[AIL_R_ID]=-120 -300; //this is always changed to a random number for no reason
    servo_offset[AIL_L_ID]=-240 +300;
    servo_offset[RUDDER_ID]=240;
    servo_offset[ELEVATOR_ID]=-80 + 230; //offset to zero servo, + offset to make elevator zero
    servo_offset[THROTTLE_ID]=0;

    servo_scaling[AIL_R_ID]=100;
    servo_scaling[AIL_L_ID]=100;
    servo_scaling[RUDDER_ID]=130;
    servo_scaling[ELEVATOR_ID]=120;
    servo_scaling[THROTTLE_ID]=100;

    ServoUpdateFlags = 1+2+4+8+16; //start the UpdateFlags as 1
    for (int i = 0; i++; i<6){
        servoPos[i] = 1500;
    }
}

void Airplane::desired_angle()
{
    //calculate the desired angle
    //For now, assume desired angle is ALWAYS VERTICAL
   X.angle_desire.w = 0.7071; //set desired angle as vertical
   X.angle_desire.x = 0.7071;
   X.angle_desire.y = 0;
   X.angle_desire.z = 0;
   X.angle_desire.normalize();  //normalize the quaternion
}

void Airplane::update_state(){
    //update state
X.angle = dat.q;  //get angle from stored sensor value
 desired_angle();  //update the desired angle

    //Calculate the difference between current and desired angle
    //SLERP = (q1 x q0[inv] ) * q0. DIFFERENCE = q0(inv) * q1
    //Quaternion q1; //create dummy quaternion
    Quaternion q0; //create dummy quaternion
    //Quaternion q0_i; //create dummy quaternion - inverse of q0
    q0 = X.angle;
    //q1 = X.angle_desire;
    //q0_i = X.angle_d;
    q0 = q0.getConjugate(); //get inverse of q0
    X.angle_off_q = q0.getProduct(X.angle_desire);//q1); //operation: (q0[inverse])*q1
    //float dum = 2*acos(q1.w); //angle difference b/w the two
    // = q0;
    X.test_angle.x = 1;
    X.test_angle.y = 0;
    X.test_angle.z = 0;
    X.test_angle.rotate(&X.angle_off_q);
//
    X.angle_v.x = 0;
    X.angle_v.y = 1;
    X.angle_v.z = 0;
    X.angle_v.rotate(&X.angle); //convert the angle of airplane to vector form (ignores yaw)
    //find difference between current angle and desired angle
    //X.angle_off.x = 0 - X.angle_v.x;
    //X.angle_off.y = 0 - X.angle_v.y;
    //X.angle_off.z = 0;


    //Convert the SLERPed quaternion to matrix form
    //VectorFloat an; // angle (from airplane's perspective) that points up (global)
    //X.angle_off.x = 0;
    //X.angle_off.y = dum;
    //X.angle_off.z = 0;
    //X.angle_off.rotate(&q1); //rotate


    //Get the incline of the airplane
    //VectorFloat dum;
    //dum.x = 0;
    //dum.y = 1;
    //dum.z = 0;
    //dum.rotate(&X.angle); //convert angle to vector form (ignores yaw)
//
//    float angle = dum.z; //this is the z-component. Same as angle_v.z

    //get the rotations of x,y, and z vector
    X.x_vect.x = 1;
    X.x_vect.y = 0;
    X.x_vect.z = 0;
    X.x_vect.rotate(&X.angle_off_q);
    X.y_vect.x = 0;
    X.y_vect.y = 1;
    X.y_vect.z = 0;
    X.y_vect.rotate(&X.angle_off_q);
    X.z_vect.x = 0;
    X.z_vect.y = 0;
    X.z_vect.z = 1;
    X.z_vect.rotate(&X.angle_off_q);


}

int Airplane::check_batt(){
    dat.batt = analogRead(BATT_PIN)*(5000/1023.0)*6;
}

void Airplane::mode_stop(){
//If the left toggle (SF) is LOW, cut power and set flaps to zero
    for (int i = 0; i < 6; i++)
        servoPos[i] = 1500;
    servoPos[THROTTLE_ID] = 1000;
}

void Airplane::outpt(){
//    Serial.println("I'm here");
}
void Airplane::add_offset(){
    //Some values must be scaled so that a value of 10 = 1 degree
    int scale;
    //int scale = (servoPos[RUDDER_ID]-1500)*(servo_scaling[RUDDER_ID]-100)/100.0;
    //servoPos[RUDDER_ID] += scale;

    for (int i = 0; i<6; i++){
        scale = (servoPos[i]-1500)*(servo_scaling[i]-100)/100.0;
        servoPos[i] += servo_offset[i] + scale;// servo_scaling[i]/100; //send the throttle value directly to output
    }
}
void Airplane::mode_airplane(){
    //Set new positions for the servos
    //if (abs(servoPos[THROTTLE_ID] - rc.throttle) >10)
    servoPos[THROTTLE_ID] = rc.throttle; //send the throttle value directly to output
    servoPos[RUDDER_ID] = limit(rc.yaw,42);
    servoPos[ELEVATOR_ID] = limit(rc.pitch,30);
    servoPos[AIL_L_ID] = limit(rc.roll,30);
    servoPos[AIL_R_ID] = limit(3000 - rc.roll,30);
}
void Airplane::mode_heli1(){
    mode_stop(); //set all other servos to 0, otherwise they get messed up in "add_offset"
    //Try to fly vertical like a helicopter
    // Find angle offset (complete in update_state())

    // Control the rudder
    // Assume: close to vertical
    if(X.angle_v.z > SIN45){   //inclination is > than 45 degrees
        //angle_v is the vector form of airplane orientation. The z-component is inclination
        //Rotate the x-axis by the angle.
/*
        VectorFloat dum;
        dum.x = 1;
        dum.y = 0;
        dum.z = 0;
        dum.rotate(&X.angle_off_q);
*/
        //VectorFloat dum2;
        //dum2.x = 0;
        //dum2.y = 0;
        //dum2.z = 1;
        //dum2.rotate(&X.angle_off_q);
        X.x_vect;
        X.z_vect;
        float dum_angle = X.x_vect.x/sqrt(pow(X.x_vect.x,2)+ pow(X.x_vect.z,2));
        float dum2_angle = X.z_vect.x/sqrt(pow(X.z_vect.x,2)+ pow(X.z_vect.z,2));
        //Make the "rotated x-axis" stay on the x-z plane.
        float Kp = 5; //gain
        float ctrl = ((Kp * -(X.x_vect.y*dum_angle+X.z_vect.y*dum2_angle) * 500) +1500); //X.x_vect.y is between -1,1. Scale this to 1000,2000
        servoPos[RUDDER_ID] = limit(ctrl,42);
    }
    else{ // Assume: close to horizontal
        //Rotate the x-axis by the angle
        //make the "rotated x-axis" stay on the x-z plane
        mode_stop();
    }

    // Control the elevator

    // Control the ailerons
}
int Airplane::limit(int value, int deg){
    deg = abs(deg);
    if ((value - 1500) < deg*-10)
        value = deg*-10 + 1500;
    if ((value - 1500) > deg*10)
        value = deg*10 + 1500;
    return value;
}

int Airplane::control(){
    //This is the main control algorithm for the airplane. Call every cycle
    int servoPosPrev[5];


    //copy the previous servo values
    for (uint8_t i = 0; i++; i<5)
        servoPosPrev[i] = servoPos[i];



    if (rc.mode < (MODE_STOP + error_max)*5+1500)
        mode_stop();
    else if ((rc.mode < (MODE_FLIGHT + error_max)*5+1500) && (rc.mode > (MODE_FLIGHT - error_max)*5+1500))
        mode_airplane();
    else if ((rc.mode < (MODE_HELI + error_max)*5+1500) && (rc.mode > (MODE_HELI - error_max)*5+1500))
        mode_heli1();
    else
        mode_stop();

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


void Airplane::print_sensors(uint8_t select){
    //Print the selected values of sensors - indicated by bit status
    //0 - IMU ypr       bin: 1 hex: 01
    //1 - IMU acc       bin: 10 hex: 02
    //2 - IMU gyr       bin: 100 hex: 04
    //3 - Ultrasonic    bin: 1000 hex: 08
    //4 - Memory useage bin: 10000 hex: 10
    //5 - RC receiver   bin: 100000  hex: 20
    //6 - Servos        bon: 1000000   hex: 40
    //7 - Test          bin: 10000000   hex: 80
    if(select){ //if any options are selected...
        if (select & 0x01){ // IMU ypr
            Serial.print("ypr\t");
            Serial.print(dat.ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[2] * 180/M_PI);
            Serial.print("\tBattery:");
            Serial.print(dat.batt);
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
        if (select & 0x80){
            Serial.print("\t Quaternion: angle wxyz: [");
            Serial.print(X.angle.w);
            Serial.print(",");
            Serial.print(X.angle.x);
            Serial.print(",");
            Serial.print(X.angle.y);
            Serial.print(",");
            Serial.print(X.angle.z);
            Serial.print("]\t vector: [");
            //Serial.print(X.angle_v.w);
            //Serial.print(",");
            Serial.print(X.z_vect.x);
            Serial.print(",");
            Serial.print(X.z_vect.y);
            Serial.print(",");
            Serial.print(X.z_vect.z);
            Serial.print("]\tTEST: [");
            Serial.print(X.x_vect.x);
            Serial.print(",");
            Serial.print(X.x_vect.y);
            Serial.print(",");
            Serial.print(X.x_vect.z);
            Serial.print("]\t");
        }
        Serial.println("");
        //If battery voltage is below 10.500V, print warning. (under 11V = 6% batt remaining)
        //if(dat.batt < 11000)
        //{
            //Serial.print("LOW BATT");
            //Serial.print(dat.batt);
        //}
    }
}

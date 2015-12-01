#include "Airplane.h"

//MPU6050 mpu;

Airplane::Airplane(){
    //Default constructor

    sensordata dat = {}; //initialize all data values to zero
    receiver rc = {1500};
    rc.throttle = 1000;

    //set the servo offsets for each servo
    servo_offset[AIL_R_ID]=-120-300; //
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
    servoPos[5] = 1000; //set the throttle to 0

//    dat.gy_ar_index = 0;
//    dat.gy_ar_size = 10;
}
void Airplane::init()
{
 //   dat.gy_ar_index = 0;
 //   dat.gy_ar_size = N_GY_AVERAGES;
}

void Airplane::predict_servo()
{
    //predict servo position
    float dum; //dummy variable for converting servoPos to radians [rad] (desired servo position)
    for(int i = 0; i < 5; i ++)
    {
        //remove offsets
        dum = (servoPos[i]  - servo_offset[i] -1500) * 30/500.0 *(PI/180.0); //1500 = 0, remove offset, -500 = -30 deg, 500 = 30 deg, deg2rad
        if(abs(dum - X.servo_pred[i]) < abs(500*PI/180.0 * dt/1000000)) //if difference between predicted and desired is less than max rotation rate (500 deg/s) times dt
            X.servo_pred[i] = dum;
        else{
            if ((dum - X.servo_pred[i]) > 0)
                X.servo_pred[i] += 500*PI/180.0 * dt/1000000.0; //1000000 micros in 1 second
            else
                X.servo_pred[i] -= 500*PI/180.0 * dt/1000000.0;
        }
        //make sure servo prediction doesn't go beyond limit of 30 degrees
        X.servo_pred[i] = max(X.servo_pred[i],-30*PI/180.0);
        X.servo_pred[i] = min(X.servo_pred[i],30*PI/180.0);
    }
}
void Airplane::predict_gy() //284 bytes
{
    //predict the rate of rotation based on the flap angles
    float K[3]; //constant multiplier:
    K[1] = 1.918/2.0;//constant multiplier for ROLL (y axis). Divide by 2 because only half the aileron is in the prop wash stream
    K[2] = 0.8506;//constant multiplier for YAW (z axis)
    K[0] = 1.89;//constant multiplier for PITCH (x axis)

    float gy_decay = 0.95; //use this value to bring the gyro value back to zero over time

//** NOTE: Multiply by 180/pi/1.5 to compare them to the DMP gyro values
    //for absolutely no reason at all, X.gy_pred[0] crashes the program. Solution: Increase array by 1 and use gy_pred[3].
    X.gy_pred[3] -= (dt/1000000.0)*K[0]*X.servo_pred[ELEVATOR_ID]; //gy x axis, using ELEVATOR servo
    X.gy_pred[2] -= (dt/1000000.0)*K[2]*X.servo_pred[RUDDER_ID]; //gy z axis, using RUDDER servo

    //For ailerons, the motor is applying a torque (negative). Add this to the predicted gyro rate
    X.gy_pred[1] -= (dt/1000000.0)*K[1]*X.servo_pred[AIL_R_ID]; //gy y axis, using AILERON servo



    //reset to 0 if the gyro measures zero and the predicted value is more than 1deg/s (0.017 rad/s)
    //decay the predicted gyro rate if the measured gyro is 0 OR the control surface is < 1 degree
    float max_rate = 0.017*10; //max rate: 0.017 = 1 deg/sec
    float decay_angle = 0.017*4; //angle of control surface at which the rate will decay (to zero it at control surface = straight)
    if(abs(X.servo_pred[ELEVATOR_ID]) < decay_angle && (abs(X.gy_pred[3]) > 0.017*3)){
        X.gy_pred[3] *= gy_decay;
   }
    if(abs(X.servo_pred[RUDDER_ID]) < decay_angle && (abs(X.gy_pred[2]) > 0.017*3))
        X.gy_pred[2] *= gy_decay;
    if(abs(X.servo_pred[AIL_R_ID]) < decay_angle && (abs(X.gy_pred[1] + 0.017*2) > 0.017*3)) //add the 0.017*2 offset of the prop torque
        X.gy_pred[1] *= gy_decay;

    //add the motor torque offset to y-axis
    X.gy_pred[1] -= (dt/1000000.0)*0.017*2; //guess: motor is applying 2deg/s rotation

    //make sure each value is less than 180deg/s = 3.14
    for(int i = 1; i < 4; i++)
    {
        if (X.gy_pred[i] > max_rate)
            X.gy_pred[i] = max_rate;
        else if (X.gy_pred[i] < -max_rate)
            X.gy_pred[i] = -max_rate;
    }
}
/*void Airplane::average_gy()
{
    dat.gy_av.x = 0;
    dat.gy_av.y = 0;
    dat.gy_av.z = 0;
    uint8_t resetx = 0;
    uint8_t resety = 0;
    uint8_t resetz = 0;
    //Calculate the average gyro values. Reset to 0 if any value is 0
    for(int i = 0; i< dat.gy_ar_size; i++){
        dat.gy_av.x += dat.gy_array[i].x;
        dat.gy_av.y += dat.gy_array[i].y;
        dat.gy_av.z += dat.gy_array[i].z;

        if(dat.gy_array[i].x == 0) //check if there were any zero vlues
            resetx ++;
        if(dat.gy_array[i].y == 0)
            resety ++;
        if(dat.gy_array[i].z == 0)
            resetz ++;
    }
    if(resetx)
        dat.gy_av.x = 0;
    else
        dat.gy_av.x /= dat.gy_ar_size;
    if(resety)
        dat.gy_av.y = 0;
    else
        dat.gy_av.y /= dat.gy_ar_size;
    if(resetz)
        dat.gy_av.z = 0;
    else
        dat.gy_av.z /= dat.gy_ar_size;

    dat.gy_av.x /= dat.gy_ar_size;
    dat.gy_av.y /= dat.gy_ar_size;
    dat.gy_av.z /= dat.gy_ar_size;
}*/
void Airplane::desired_angle()
{
    //calculate the desired angle
    //For now, assume desired angle is ALWAYS VERTICAL
   X.angle_desire.w = 0.7071; //set desired angle as vertical
   X.angle_desire.x = 0.7071;
   X.angle_desire.y = 0;
   X.angle_desire.z = 0;
   X.angle_desire.normalize();  //normalize the quaternion

    Quaternion input;
    input.w = ((rc.roll-1500)/500.0)*5*PI/180.0;
    input.x = 0;
    input.y = 0;
    input.z = 1;
    X.angle_desire = X.angle_desire.getProduct(input);

    input.w = -1*((rc.pitch-1500)/500.0)*5*PI/180.0;
    input.x = 1;
    input.y = 0;
    input.z = 0;
    X.angle_desire = X.angle_desire.getProduct(input);

    input.w = ((rc.yaw-1500)/500.0)*15*PI/180.0;
    input.x = 0;
    input.y = 1;
    input.z = 0;
    X.angle_desire = X.angle_desire.getProduct(input);
   //add the yaw/pitch/roll from receiver
   //multiply X by yaw, then pitch, then roll
}

void Airplane::update_angle(){
    //update state
X.angle = dat.q;  //get angle from stored sensor value
 desired_angle();  //update the desired angle

    //Calculate the difference between current and desired angle
    //Quaternion q1; //create dummy quaternion
    Quaternion q0; //create dummy quaternion
    //Quaternion q0_i; //create dummy quaternion - inverse of q0
    q0 = X.angle;
    //q1 = X.angle_desire;
    //q0_i = X.angle_d;
    q0 = q0.getConjugate(); //get inverse of q0
    X.angle_off_q = q0.getProduct(X.angle_desire);//q1); //operation: (q0[inverse])*q1. This gets the difference between q0 and q1 //takes 250us
//

    q0.w = 0; //save 172 bytes by not using rotate function
    q0.x = 0;
    q0.y = 1;
    q0.z = 0;
    q0 = X.angle.getProduct(q0);
    q0 = q0.getProduct(X.angle.getConjugate());
    X.angle_v.x = q0.x;
    X.angle_v.y = q0.y;
    X.angle_v.z = q0.z;
    /*
    X.angle_v.x = 0;
    X.angle_v.y = 1;
    X.angle_v.z = 0;
    X.angle_v.rotate(&X.angle); //convert the angle of airplane to vector form (ignores yaw) //'rotate' takes 300us
*/
    //get the rotations of x,y, and z vector
    //save 400 bytes by not using the ROTATE function (it redeclares one quaternion per use)
    q0.w = 0;
    q0.x = 1;
    q0.y = 0;
    q0.z = 0;
    q0 = X.angle_off_q.getProduct(q0);
    q0 = q0.getProduct(X.angle_off_q.getConjugate());
    X.x_vect.x = q0.x;
    X.x_vect.y = q0.y;
    X.x_vect.z = q0.z;

    q0.w = 0;
    q0.x = 0;
    q0.y = 1;
    q0.z = 0;
    q0 = X.angle_off_q.getProduct(q0);
    q0 = q0.getProduct(X.angle_off_q.getConjugate());
    X.y_vect.x = q0.x;
    X.y_vect.y = q0.y;
    X.y_vect.z = q0.z;

    q0.w = 0;
    q0.x = 0;
    q0.y = 0;
    q0.z = 1;
    q0 = X.angle_off_q.getProduct(q0);
    q0 = q0.getProduct(X.angle_off_q.getConjugate());
    X.z_vect.x = q0.x;
    X.z_vect.y = q0.y;
    X.z_vect.z = q0.z;
    /*X.x_vect.x = 1;
    X.x_vect.y = 0;
    X.x_vect.z = 0;
    X.x_vect.rotate(&X.angle_off_q); //'rotate' takes 300us and 1000bytes
    X.y_vect.x = 0;
    X.y_vect.y = 1;
    X.y_vect.z = 0;
    X.y_vect.rotate(&X.angle_off_q); //'rotate' takes 300us
    X.z_vect.x = 0;
    X.z_vect.y = 0;
    X.z_vect.z = 1;
    X.z_vect.rotate(&X.angle_off_q); //'rotate' takes 300us*/


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
    servoPos[THROTTLE_ID] = limit(rc.throttle,50); //send the throttle value directly to output
    servoPos[RUDDER_ID] = limit(rc.yaw,42);
    servoPos[ELEVATOR_ID] = limit(rc.pitch,30);
    servoPos[AIL_L_ID] = limit(rc.roll,30);
    servoPos[AIL_R_ID] = limit(3000 - rc.roll,30);
}
void Airplane::mode_heli1(){
    mode_stop(); //set all other servos to 0, otherwise they get messed up in "add_offset"
    //Try to fly vertical like a helicopter
    // Find angle offset (complete in update_state())

    //Set the desired angle to vertical
    //Rotate the desired angle by the input from controller

    // Control the rudder
    // Assume: close to vertical
    if(X.angle_v.z > SIN45){   //inclination is > than 45 degrees
        //RUDDER
        //angle_v is the vector form of airplane orientation. The z-component is inclination
        //Rotate the x-axis by the angle.
        float dum = sqrt(pow(X.x_vect.x,2)+ pow(X.x_vect.z,2));   //This approaches 1 when the axis is parallel to x-axis. 0 if parallel to z-axis ORIGINAL
        float dum2 = sqrt(pow(X.z_vect.x,2)+ pow(X.z_vect.z,2));  //ORIGINAL
        //Make the "rotated x-axis" stay on the x-z plane.
        float Kp = 15; //proportional gain
        float Kd = 17; //derivative gain
        float Ki = 0; //integral gain
        float ctrl = Kp * -(X.x_vect.y*X.x_vect.x/dum+X.z_vect.y*X.z_vect.x/dum2); //X.x_vect.y is between -1,1. Scale this to 1000,2000
        //Derivative Control - rate of rotation in the z-axis
        //Get the rate of rotation in z-axis. use gy_av for the average rate of rotation (because min sensitivity of sensor is 3deg/s0
//        ctrl = ctrl + Kd*dat.gy_av.z*(1.5/1.0)*(PI/180.0); //add the derivative control (or more precisely: subtract it) 32768 is equal to 250 deg/second rotation. 1deg = pi/180 rad
        ctrl = ctrl + Kd*X.gy_pred[2];
        ctrl = ctrl*500 + 1500; //Shift to between 1000,2000
        servoPos[RUDDER_ID] = limit(ctrl,42);

        //ELEVATOR - same as rudder, but x and z swapped in 'x_vect.x' > 'x_vect.z'
        ctrl = Kp * (X.x_vect.y*X.x_vect.z/dum+X.z_vect.y*X.z_vect.z/dum2); //proportional
        //ctrl = ctrl + Kd * dat.gy_av.x*(1.5/1.0)*(PI/180.0);                                      //derivative
        ctrl = ctrl + Kd * X.gy_pred[3];  //derivative - predicted gyro in the x-axis
        ctrl = ctrl*500 + 1500;
        servoPos[ELEVATOR_ID] = limit(ctrl,30);

        //AILERONS - fix the roll.
        Kp = 7;
        Kd = 15;
        ctrl = Kp * X.x_vect.z; //if this is -ve, it flips the plane's orentation by 180 degrees. Double check in case it wants to fly upside-down.
        //ctrl = ctrl + Kd * dat.gy_av.y*(1.5/1.0)*(PI/180.0);
        ctrl = ctrl + Kd * X.gy_pred[1];
        ctrl = ctrl*500 + 1500;
        servoPos[AIL_L_ID] = limit(ctrl,30);
        servoPos[AIL_R_ID] = limit(ctrl,30);

        //get the throttle from receiver
        servoPos[THROTTLE_ID] = limit(rc.throttle,50);
    }
    else{ // Assume: close to horizontal
        //Use rudder to fix yaw - keep the

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
    //update the time
    dt = micros() - t_prev;
    if(dt > 15000)
        dt = 0; //reset to zero if hte micros() counter rolls over
    predict_servo(); //predict the current servo position [rad]
    predict_gy();   //predict the current gyro rate [rad/s]*10000
    t_prev = micros();
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
    //if(select){ //if any options are selected...
/*        if (select & 0x01){ // IMU ypr
            Serial.print("ypr\t");
            Serial.print(dat.ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(dat.ypr[2] * 180/M_PI);
            Serial.print("\tBattery:");
            Serial.print(dat.batt);
        }*/
/*        if (select & 0x02){  //116 bytes
            Serial.print("acc\t");
            Serial.print(dat.aa.x);
            Serial.print("\t");
            Serial.print(dat.aa.y);
            Serial.print("\t");
            Serial.print(dat.aa.z);
            Serial.print("\t");
        }
*/       if (select & 0x04){ //274 bytes
            Serial.print("gy\t");
            Serial.print(dat.gy.x);
            Serial.print("\t");
            Serial.print(dat.gy.y);
            Serial.print("\t");
            Serial.print(dat.gy.z);
            Serial.print("\t Gyroangle:");
            Serial.print(anglegyro);
            Serial.print("\t avg xyz:");
            /*Serial.print(X.gy_pred[3]);
            Serial.print("\t");
            Serial.print(X.gy_pred[1]);
            Serial.print("\t");
            Serial.print(X.gy_pred[2]);*/
        }
/*        if (select & 0x08){
            Serial.print("UltraB: ");
            Serial.print(dat.ult_b);
            Serial.print("\t UltraR: ");
            Serial.print(dat.ult_r);
        }
        if (select & 0x10){     //300us
            Serial.print("\t mem: ");
            Serial.print(freeMemory());
        }
 /*       if (select & 0x20){
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
 */       if (select & 0x40){     //264 bytes
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
            Serial.print("\t Pred Ail:");
            Serial.print(X.servo_pred[AIL_L_ID]);
        }
 /*       if (select & 0x80){ //4700us
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
        }*/
        Serial.println("");
        //If battery voltage is below 10.500V, print warning. (under 11V = 6% batt remaining)
        //if(dat.batt < 11000)
        //{
        //    Serial.print("LOW BATT");
        //    Serial.print(dat.batt);
        //}
    //}
}

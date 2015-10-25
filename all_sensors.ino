/* ===========================================
This code is based off of Jeff Rowberg's MPU6050 dmp example.
It is modified for my personal helicoptor project (not finished yet).
Author: Mark vandermeulen

==============================================
*/
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*==============================================
HARDWARE FIXES:
Ultrasonic: Add a 2.2k R between trig and echo for 1-pin operation. Add a 0.1uF cap in series seems to help.
            Pin D10 (ult_rear) seems to work a bit better than D9 (ult_bottom).

SOFTWARE FIXES:
RCArduinoFastLib: RC_CHANNEL_OUT_COUNT = #of servos on bank + 1 for the frame = 4+1 = 5
                  attach(ch,pin#). Use ch 0,1,2,3 for servos.
                  Set frame: use setFrameSpaceA(ch,us) to make the total update rate to 50Hz for servos
                  For 2nd bank of servos: when using attach, set the ESC ID = RC_CHANNEL_OUT_COUNT = 5.
                    This makes the ESC as channel 0 on bank B.
                    Then you must add 4 blank channels to bank B (I used the "setFrameSpaceB()" 4 times
                    using channels 1,2,3,4.
================================================
*/


#include "all_sensors.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;


//Flag for updating servos. This is used in this file and Airplane.cpp
volatile uint8_t ServoUpdateFlags; //Should be volatile b/c it's used in 2 separate files

bool dmpReady = false;  // Flag for MPU6050
Airplane a;             // Custom class

// ================================================================
// ===           INTERRUPT DETECTION ROUTINES MUST BE IN MAIN   ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    //** MAKE SURE that the RECEIVER interrupts don't
    mpuInterrupt = true;
}


//ISR for ultrasonic sensor. Must use separate ISR for each sensor. Must not run simultaneously
void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // ult_b&r are single-byte vars, so they don't need read/write protection. http://forum.arduino.cc/index.php?topic=45239.0
  //56 us if there is trigger, 12 us if not.
  if (ultra_bot.check_timer()) { // This is how you check to see if the ping was received.
       a.dat.ult_b = ultra_bot.ping_result / US_ROUNDTRIP_CM;
  }
}
void echoCheck_r() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (ultra_rear.check_timer()) { // This is how you check to see if the ping was received.
       a.dat.ult_r = ultra_rear.ping_result / US_ROUNDTRIP_CM;
  }
}

// simple interrupt service routine for RECEIVER
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}
void calcYaw()
{
  if(PCintPort::pinState)
  {
    unYawInStart = TCNT1;
  }
  else
  {
    unYawInShared = (TCNT1 - unYawInStart)>>1;
    bUpdateFlagsShared |= YAW_FLAG;
  }
}
void calcPitch()
{
  if(PCintPort::pinState)
  {
    unPitchInStart = TCNT1;
  }
  else
  {
    unPitchInShared = (TCNT1 - unPitchInStart)>>1;
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}
void calcRoll()
{
  if(PCintPort::pinState)
  {
    unRollInStart = TCNT1;
  }
  else
  {
    unRollInShared = (TCNT1 - unRollInStart)>>1;
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}
void calcMode()
{
  if(PCintPort::pinState)
  {
    unModeInStart = TCNT1;
  }
  else
  {
    unModeInShared = (TCNT1 - unModeInStart)>>1;
    bUpdateFlagsShared |= MODE_FLAG;
  }
}





// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
dmpReady = false;

a.control();
//    a.servo_set();

     // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize serial communication
    Serial.begin(115200);

        //while (!Serial); // wait for Leonardo enumeration, others continue immediately
delay(100);
    initialize_imu();


    pingTimer = millis(); //this is used for ultrasonic sensors

    /////////////// Initialize SERVOS and RECEIVER //////////////////////
    // attach servo objects, these will generate the correct
    // pulses for driving Electronic speed controllers, servos or other devices
    // designed to interface directly with RC Receivers
    CRCArduinoFastServos::attach(THROTTLE_ID,THROTTLE_OUT_PIN); //throttle servo
    CRCArduinoFastServos::attach(AIL_R_ID,AIL_R_OUT_PIN);       //aileron servo
    CRCArduinoFastServos::attach(AIL_L_ID,AIL_L_OUT_PIN);       //aileron servo
    CRCArduinoFastServos::attach(RUDDER_ID,RUDDER_OUT_PIN);       //rudder servo
    CRCArduinoFastServos::attach(ELEVATOR_ID,ELEVATOR_OUT_PIN);       //elevator servo

    // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 4 Servos (4*2000) + 6 times 2000
    CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,12000); //20000 - 4servos*2000 = 12000 // 50Hz
    //For bank B, we need the same number of channels as bank A (4 SERVO + 1 frame)channels. Since we only have ESC,
    //bank B will have (1 ESC + 4 frame)channels.
    CRCArduinoFastServos::setFrameSpaceB(1,1*100); // Frame space for bank B (4 blank channels so total is 5)
    CRCArduinoFastServos::setFrameSpaceB(2,1*100);
    CRCArduinoFastServos::setFrameSpaceB(3,1*100);
    CRCArduinoFastServos::setFrameSpaceB(4,17*100);

    CRCArduinoFastServos::begin();
    // using the PinChangeInt library, attach the interrupts
    // used to read the channels
    PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
    PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
    PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
    PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);
    PCintPort::attachInterrupt(MODE_IN_PIN, calcMode,CHANGE);

    //update_servos();


//delay(100);
    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);

    Serial.print("Free Memory: ");
    Serial.println(freeMemory());

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    unsigned long time = 0;
    time = micros();

    int c = 0;
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;
c = 0;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt){// && fifoCount < packetSize) { //The fifocount thing here crashes the code after a while
      //time = micros();
      //Serial.print(micros()-time);
        //Serial.println(" <- time");

        // ULTRASONIC: send another ping if 50ms has passed since last
/*
        if (millis() >= pingTimer){ //4us if all false. 500us if trigger.
            if (ULTRA_SELECT){  //swap between checking bottom and rear ultrasonic
                ultra_bot.ping_timer(echoCheck);
                pingTimer += pingSpeed;
                ULTRA_SELECT = 0;
            }else if (!ULTRA_SELECT){
                ultra_rear.ping_timer(echoCheck_r);
                pingTimer += pingSpeed;
                ULTRA_SELECT = 1;
            }
        }

        //check if the ultrasonics see nothing
        if (!ultra_bot.ping_result){
            a.dat.ult_b = 0;
        }
        if (!ultra_rear.ping_result){
            a.dat.ult_r = 0;
        }
*/
        //delay(100);
        update_receiver();
        a.control();
        update_servos();

}
//Serial.print(c);
// use if(mpuInterrupt && dmpReady) instead of while to update imu stuff.
// check the mpuInt before each operation of non-critical things.
// use flags and stuff for the ultrasonics (needs an Interrupt routine)
// check the radio inputs (use an ISR)
// control algorithm - check mode flags
// motor and servo controls
// write to sd
// time each section

update_imu();  //this function at the bottom of the file
}

int update_imu(){  //there are linker errors if I put this fn in a separate file
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        // get quat, accel, and gy (raw values)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(&gy, fifoBuffer);

        //save the quaternion, accel, and gyr
        a.dat.q = q;
        a.dat.aa = aa;
        a.dat.gy = gy;

        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        a.dat.ypr[0] = ypr[0];
        a.dat.ypr[1] = ypr[1];
        a.dat.ypr[2] = ypr[2];

        //Serial.print("TIME: "); // Print a recorded delta time
        //Serial.print(a.dat.dt); //
        a.print_sensors(0x20); //eventually move this into main loop
    }
}

int initialize_imu(){
     // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
 //       Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
//      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

// Must be here because it deals with interrupts?
void update_receiver(){
    // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      a.rc.throttle = unThrottleInShared;
    }

    if(bUpdateFlags & YAW_FLAG)
    {
      a.rc.yaw = unYawInShared;
    }
    if(bUpdateFlags & PITCH_FLAG)
    {
      a.rc.pitch = unPitchInShared;
    }
    if(bUpdateFlags & ROLL_FLAG)
    {
      a.rc.roll = unRollInShared;
    }
    if(bUpdateFlags & MODE_FLAG)
    {
      a.rc.mode = unModeInShared;
    }
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
}

// Linker errors if RCArduinoFastServos is in a separate file. Must be placed here.
void update_servos(){
  if(ServoUpdateFlags & SERVO_FLAG_THROTTLE)
  {
    CRCArduinoFastServos::writeMicroseconds(THROTTLE_ID,a.servoPos[THROTTLE_ID]);//servoPos[THROTTLE_ID]);
    ServoUpdateFlags = ServoUpdateFlags & (~SERVO_FLAG_THROTTLE); //clear the throttle flag to show it was updated
  }
  if(ServoUpdateFlags & SERVO_FLAG_RUD)
  {
      CRCArduinoFastServos::writeMicroseconds(RUDDER_ID,a.servoPos[RUDDER_ID]); //
      ServoUpdateFlags = ServoUpdateFlags & (~SERVO_FLAG_RUD); //clear the rudder update flag
  }
  if(ServoUpdateFlags & SERVO_FLAG_ELEV)
  {
      CRCArduinoFastServos::writeMicroseconds(ELEVATOR_ID,a.servoPos[ELEVATOR_ID]); //
      ServoUpdateFlags = ServoUpdateFlags & (~SERVO_FLAG_ELEV); //clear the rudder update flag
  }
  if(ServoUpdateFlags & SERVO_FLAG_R)
  {
      CRCArduinoFastServos::writeMicroseconds(AIL_R_ID,a.servoPos[AIL_R_ID]); //
      ServoUpdateFlags = ServoUpdateFlags & (~SERVO_FLAG_R); //clear the rudder update flag
  }
  if(ServoUpdateFlags & SERVO_FLAG_L)
  {
      CRCArduinoFastServos::writeMicroseconds(AIL_L_ID,a.servoPos[AIL_L_ID]); //
      ServoUpdateFlags = ServoUpdateFlags & (~SERVO_FLAG_L); //clear the rudder update flag
  }
}

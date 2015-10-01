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

#include "all_sensors.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

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


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
dmpReady = false;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
a.control();
//    a.servo_set();
    // initialize serial communication
    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
delay(100);
    initialize_imu();

delay(100);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    Serial.print("Free Memory: ");
    Serial.println(freeMemory());

    pingTimer = millis(); //this is used for ultrasonic sensors
    //
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    unsigned long time = 0;
    time = micros();

    int c = 0;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
c = 0;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt){// && fifoCount < packetSize) { //The fifocount thing here crashes the code after a while
      //time = micros();
      c+=1; //~1 us
      //Serial.print(micros()-time);
        //Serial.println(" <- time");
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .

        // ULTRASONIC: send another ping if 50ms has passed since last

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
        //delay(100);
        a.control();
//        a.servo_set();
        //for (int i = 0; i++;i<5)
        //    a.servos[i].refresh();

  //SoftwareServo::refresh();
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
        a.print_sensors(0x09); //eventually move this into main loop
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

#ifdef SD_CARD_USED
int initialize_SD(){  //Mysterious failure when this function is in a different file
       // == SD CARD INIT == //
    Serial.print("Initializing SD card...");
    pinMode(SS, OUTPUT);
    if (!SD.begin(10)) {
        Serial.println("initialization failed!");
        return 1;
    }
    //Serial.println("SD initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  a.myFile = SD.open("datalog/test2.txt", FILE_WRITE);
    // if the file opened okay, write to it:
  if (a.myFile) {
    Serial.print("Writing to test2.txt...");
    a.myFile.println("testing 1, 2, 3.");
	// close the file:
    a.myFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  // re-open the file for reading:
  a.myFile = SD.open("datalog/test2.txt");
  if (a.myFile) {
    Serial.println("test2.txt:");

    // read from the file until there's nothing else in it:
    while (a.myFile.available()) {
    	Serial.write(a.myFile.read());
    }
    // close the file:
    a.myFile.close();
  } else {
  	// if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
#endif

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

  //NewPing ultra_bot(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);// = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  //NewPing ultra_rear(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);// = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  //pingTimer = millis(); //start timer for ultrasonics
}

void Airplane::outpt(){
//    Serial.println("I'm here");
}





#ifdef SD_CARD_USED
void Airplane::SD_close(){ //close current log file
    myFile.close();
}
//Create a new log file.
//Find the name. Open file. Increment counter. DO NOT CLOSE
int Airplane::SD_newLog(){
    char name[] = "0000x000.txt"; //create name for file
    uint16_t dum1 = flight_index;
    uint8_t dum2 = log_index;

    uint8_t digits = 1;

    //for the flight index number
    while (dum1/=10)   //find  number of digits
        ++digits;
    dum1 = flight_index;

    //modify the name to fit with flight_index and log_index
    for (uint8_t i = 0; i++; i<digits)
    {
        uint8_t j = 1;
        while(j<digits)
            dum1/=10;
        name[3-i] = dum1%10 + 48; //add 48 to convert int to char
    }

    //for the log index number
    digits = 1;
    while (dum2/=10)   //find  number of digits
        ++digits;
    dum2 = log_index;
    //modify the name to fit with flight_index and log_index
    for (uint8_t i = 0; i++; i<digits)
    {
        uint8_t j = 1;
        while(j<digits)
            dum1/=10;
        name[3-i] = dum1%10 + 48; //add 48 to convert int to char
    }


    Serial.println("New log file:");
    myFile = SD.open(name, FILE_WRITE);

    if (myFile) {
    //Serial.print("Writing to newfile.txt...");
    myFile.println("testing 1.");
	// close the file:
    myFile.close();
    Serial.println("done.");
    } else {
    // if the file didn't open, print an error:
    Serial.println("error opening newfile.txt");
  }
  // re-open the file for reading:
  myFile = SD.open(name);
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
    	Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
  	// if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  log_index ++;
}
//Index log: create index if none exists
//Else, find value of previous index, add new flight index. WORKING
int Airplane::index_log(){
    char dum;
    flight_index = 0; //initialize to 0 so index start at zero if no index_log.txt

    myFile = SD.open("datalog/index.txt"); //open file, create new

    if(myFile){ //IF FILE EXISTS, READ THE LAST LINE OF INDEX
        while (myFile.available()) {
            dum =  myFile.read();   //read one byte
            //Serial.print(dum);      //print it to screen
            if (dum < 48)  //if it reads a command char, reset flight index
                flight_index = 0;
            else
                flight_index = (flight_index * 10) + (dum-48); //add the chars together
        }
        flight_index++; //increment the flight index
        if (flight_index > 9999) //make sure flight_index is 4 digits
            flight_index = 0;
        //Serial.print("value of index: ");
        //Serial.print(flight_index);
    }
    myFile.close(); //close the file

    //WRITE TO FILE
    myFile = SD.open("datalog/index.txt", FILE_WRITE); //also creates file if not exist
if (myFile) {
    //Serial.print("Writing to newfile.txt...");
    myFile.println(""); //carriage return
    myFile.print(flight_index);
	// close the file:
    myFile.close();
    } else {
        // if the file didn't open, print an error:
        //Serial.println("error opening index.txt");
        return -1;
  }
  myFile.close(); //close the file
  return 0; //return no error
}
#endif

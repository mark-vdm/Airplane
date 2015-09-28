#include "Airplane.h"

//MPU6050 mpu;

Airplane::Airplane(){
    //Default constructor
//    blinkState = false;
    // MPU control/status vars
  //  dmpReady = false;
  record_index = 0;
}

void Airplane::outpt(){
    Serial.println("I'm here");
}

int Airplane::SD_newLog(){
    Serial.println("New log file:");
    myFile = SD.open("newfile.txt", FILE_WRITE);

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
  myFile = SD.open("newfile.txt");
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
}

//Index log: create index if none exists
//Else, add a new index. WORKING
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

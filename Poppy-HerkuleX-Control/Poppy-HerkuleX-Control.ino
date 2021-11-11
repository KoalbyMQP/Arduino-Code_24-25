#include <Herkulex.h>

enum Commands {Init = 0, GetPosition = 1, SetPosition = 2, SetPositionT = 3};//TODO - switch to an enum-based setup when testing with physical robot

int idArr[][4] ={{0x01, -10,-100,   -70}, //0
                 {0x02, -25,70,     60},   //1
                 {0x03, -100,100,   0}, //2 TEMP
                 {0x0F, -100,100,   0}, //3 TEMP
                 {0x07, -60,60,     0},   //4 TEMP
                 {0x06, -100,100,   0}, //5 TEMP
                 {0x0B, -100,100,   0}, //6 TEMP
                 {0x0A, -100,100,   0}, //7 TEMP
                 {0x12, -34,58,     0},   //8
                 {0x11, -163,-17,   -80},  //9
                 {0x13, -166,166,   0}  //10
                 }; 

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1 
  delay(500); 
  //Herkulex.initialize(); //initialize motors
  delay(200);  
  pinMode(A0,INPUT);
}

void loop(){
  // send data only when you receive data:

  if(Serial.available()!=0){
    int cmd = Serial.parseInt();
    Serial.print("Command: ");
    Serial.println(cmd);
    if(cmd == Init){//Reset all motors
      Herkulex.initialize(); //initialize motors
      for(int i = 0;i<sizeof(idArr);i++)
        Herkulex.reboot(idArr[i][0]);
      delay(200);
      //Move each motor to initialized position
      for(int i = 0;i<sizeof(idArr);i++){
        Herkulex.torqueON(idArr[i][0]);
        Herkulex.moveOneAngle(idArr[i][0], idArr[i][2], 1000, LED_BLUE);
      }
      
    }else if(cmd == GetPosition){//TODO - return motor position
      Serial.println("Motor position here");
    }else if (cmd == SetPosition){
      //Serial.println("Enter Motor Index ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int motorNum = Serial.parseInt();                    //Read user input and hold it in a variable
     
      // Print well formatted output
      Serial.print("Recieved: ");                 
      Serial.println(motorNum);
      
      Serial.println("Enter Motor Position ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int positionPerc = Serial.parseInt();                    //Read user input and hold it in a variable
     
      // Print well formatted output
      Serial.print("Recieved: ");                 
      Serial.println(positionPerc);
      
      Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc,0,100,idArr[motorNum][1],idArr[motorNum][2]), 1000, LED_BLUE); //move motor with 300 speed
    }else if (cmd == SetPositionT){ //Set position with time of motion
      //Serial.println("Enter Motor Index ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int motorNum = Serial.parseInt();                    //Read user input and hold it in a variable
     
      // Print well formatted output
      Serial.print("Recieved: ");                 
      Serial.println(motorNum);
      
      Serial.println("Enter Motor Position ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int positionPerc = Serial.parseInt();                    //Read user input and hold it in a variable

      Serial.println("Enter travel time (millis) ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int tTime = Serial.parseInt();                    //Read user input and hold it in a variable
     
      // Print well formatted output
      Serial.print("Recieved: ");                 
      Serial.println(positionPerc);
      
      Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc,0,100,idArr[motorNum][1],idArr[motorNum][2]), tTime, LED_BLUE); //move motor with 300 speed
    }
  }


  delay(50); 
}

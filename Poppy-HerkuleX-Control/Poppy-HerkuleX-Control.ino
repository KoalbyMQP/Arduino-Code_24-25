#include <Herkulex.h>

//List of possible numeric commands to send
enum Commands { Init = 0, GetPosition = 1, SetPosition = 2,
                SetPositionT/*Set position with ID and time of motion*/ = 3,
                armMirror = 4};//TODO - switch to an enum-based setup when testing with physical robot


               //  ID    Limits (low & high)  Home Position
int idArr[][4] ={{0x01,   -3,-145,           -5}, //0
                 {0x02,   -56,126,             30},   //1
                 {0x03,   -16,160,           0}, //2 TEMP
                 {0x0F,   107,-15,           70}, //3 TEMP
                 {0x07,   13,143,             55},   //4 R shoulder
                 {0x06,   13,-160,           0}, //5 Arm R
                 {0x0B,   -3,124,           2}, //26 Arm R
                 {0x0A,   97,166,           97}, //7 Arm R
                 {0x12,   -30,55,             20},   //8
                 {0x11,   -163,-17,           -94},  //9
                 {0x13,   -166,166,           0}  //10
                 }; 
int mirrorArray[4][2] = {{3,4},
                         {2,5},
                         {1,7},
                         {0,6}};

bool armMirrorModeOn = false;
int lastMirror = 0;
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
  //See if command availible - if not loop again, this allows for various maintainence tasks
  if(Serial.available()!=0){
    int cmd = Serial.parseInt();
    Serial.print("Command: ");
    Serial.println(cmd);
    if(cmd == Init){//Reset all motors
      Herkulex.initialize(); //initialize motors
      for(int i = 0;i<sizeof(idArr);i++)
        Herkulex.reboot(idArr[i][0]);
      delay(500);
      //Move each motor to initialized position
      for(int i = 0;i<sizeof(idArr);i++){
        Herkulex.torqueON(idArr[i][0]);
        Herkulex.moveOneAngle(idArr[i][0], idArr[i][3], 1000, LED_BLUE);

        //I cannot explain why this line is needed, but I swear on my life removing it makes the motor stop working right in init
        Serial.println(Herkulex.getAngle(idArr[i][0]));
      }
      
    }else if(cmd == GetPosition){//TODO - return motor position
      Serial.println("Motor position here");
    }else if (cmd == SetPosition){ //Set position, use default time of motion
       //Read motor number
      Serial.println("Enter Motor Index ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int motorNum = Serial.parseInt();                    //Read user input and hold it in a variable 

      //Read motor target position
      Serial.println("Enter Motor Position ");
      while (Serial.available()==0){} 
      int positionPerc = Serial.parseInt();

      //Send parsed command to the motor
      Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc,0,100,idArr[motorNum][1],idArr[motorNum][2]), 1000, LED_BLUE); //move motor with 300 speed
    }else if (cmd == SetPositionT){ //Set position with time of motion
      //Read motor number
      Serial.println("Enter Motor Index ");     
      while (Serial.available()==0){} 
      int motorNum = Serial.parseInt();  

      //Read motor target position
      Serial.println("Enter Motor Position ");
      while (Serial.available()==0){} 
      int positionPerc = Serial.parseInt();

      //Read time of motion
      Serial.println("Enter travel time (millis) ");
      while (Serial.available()==0){}
      int tTime = Serial.parseInt();
      
      //Send parsed command to the motor
      Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc,0,100,idArr[motorNum][1],idArr[motorNum][2]), tTime, LED_BLUE); //move motor with 300 speed
    }else if (cmd == armMirror){ //Set position, use default time of motion
       //Read motor number
      Serial.println("Enter Value (0 off, 1 on) ");        //Prompt User for input
      while (Serial.available()==0){}             // wait for user input
      int motorNum = Serial.parseInt();  

      if(motorNum){
        armMirrorModeOn = true;
        for(int i =0; i<4;i++){
          Herkulex.torqueOFF(idArr[mirrorArray[i][0]][0]);
        }
      }else{
        armMirrorModeOn = false;
        for(int i =0; i<4;i++){
          Herkulex.torqueON(idArr[mirrorArray[i][0]][0]);
        }
      }
    }
  }

  //Do maintence stuff here - ex: balancing the robot if IMU attached to Arduino
  //....

  if(armMirrorModeOn){
    lastMirror = millis();
    for(int i =0; i<4;i++){
       int rowRead =mirrorArray[i][0];
       int rowSet = mirrorArray[i][1];
       int pos1 = Herkulex.getAngle(idArr[rowRead][0]);

       //Serial.println(map(pos1,idArr[rowRead][1],idArr[rowRead][2],idArr[rowSet][1],idArr[rowSet][2]));
      
       Herkulex.moveOneAngle(idArr[rowSet][0], map(pos1,idArr[rowRead][1],idArr[rowRead][2],idArr[rowSet][1],idArr[rowSet][2]), 500, LED_BLUE); //move motor
    }

    //delay(100);
  }
 delay(20); 
}


#include "DFRobot_DF2301Q.h"

#define TRIGGER_PIN 9

#define WAKEUP_WORD_LEARNING_ID 1  // "Wake-up words for learning"
#define HELLO_ROBOT_ID         2  // "Hello robot"

DFRobot_DF2301Q_I2C asr;

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  while (!(asr.begin())) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  // Set voice volume (1~7)
  asr.setVolume(4);

  // Enable audio feedback (if desired)
  asr.setMuteMode(0);

  // Set wake-up duration (0-255)
  asr.setWakeTime(20);

  // Confirm wake-up duration
  uint8_t wakeTime = asr.getWakeTime();
  Serial.print("wakeTime = ");
  Serial.println(wakeTime);

  
  delay(300);
}
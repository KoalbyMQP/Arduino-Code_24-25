/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


#include <DynamixelShield.h>

#define DEBUG_SERIAL Serial2

const uint8_t DXL_ID = 0x03;
const uint8_t DXL_ID2 = 0x04;

const float DXL_PROTOCOL_VERSION = 1.0;

//2 ,146, 77

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  // Swap 1 to 2
  DEBUG_SERIAL.print("Protocol 1 to 2: ");
  int t=millis();
  dxl.setPortProtocolVersion(1.0);
  int tm = millis()-t;
  DEBUG_SERIAL.println(tm);
  
  //Message 2
  DEBUG_SERIAL.print("Message on 2: ");
  t=millis();
  dxl.setGoalPosition(DXL_ID2, 150, UNIT_DEGREE);
  tm = millis()-t;
  DEBUG_SERIAL.println(tm);

  //Swap 2 to 1
  DEBUG_SERIAL.print("Protocol 2 to 1: ");
  t=millis();
  dxl.setPortProtocolVersion(2.0);
  tm = millis()-t;
  DEBUG_SERIAL.println(tm);

  //Message 1
  DEBUG_SERIAL.print("Message on 1: ");
  t=millis();
  dxl.setGoalPosition(DXL_ID, 150, UNIT_DEGREE);
  tm = millis()-t;
  DEBUG_SERIAL.println(tm);

  //Add empty line
  DEBUG_SERIAL.print("\n");
  delay(1000);
}

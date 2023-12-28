#include <mcp_can.h>
#include <SPI.h>

unsigned long previousMillis = 0;
long interval = 200;                        //sendData interval in milliseconds
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
bool keyOn = false;
bool debug = true;                          // set to true to print out read messages in readCAN function
bool brake_pedal_pressed = false;
long int counter = 0;

//CAN0 is output to IPC
//CAN1 is input from BCM
//CAN2 is input from platform
//CAN3 is input from Transit

#define CAN0_INT 8                      //set interrupt pins for reading CAN
#define CAN1_INT 2                             
#define CAN2_INT 4
#define CAN3_INT 6

MCP_CAN CAN0(A0);                        // Set CS pins
MCP_CAN CAN1(3);
MCP_CAN CAN2(5);
MCP_CAN CAN3(7);

void setup()
{
  Serial.begin(115200);
  delay(3000);          //wait for board startup

  // Initialize all CAN boards @ 500kb/s no masks or filters.
  // Use 8MHz for MCP2515 boards in use
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN0 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN0...");

  if(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN1...");

  if(CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN1...");

  if(CAN3.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN1...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  CAN1.setMode(MCP_NORMAL);
  CAN2.setMode(MCP_NORMAL);
  CAN3.setMode(MCP_NORMAL);
}

byte backlight = 0x10;                // 0x00 = off, 0x10 = on
byte parking_brake_light = 0x00;      // 0x00 = off, 0xC0 = on
byte speed = 0x00;                    // speed from 0x00 - 0x40(64)
byte rpm = 0x00;                      // rpm from 0x00 - 0x60(96)
byte turn_signal = 0x48;              // 0x_5 = right, 0x_6 = left, 0x_7 both
byte abs_light = 0x00;                // 0x00 = off, 0x01 = solid, 0x02= fast blink, 0x03 = slow blink
byte stability_control = 0x00;        // 0x00 = off, 0x50 = solid, 0x90 = slow blink, 0xF0 = fast blink
byte trans_temp = 0x40;               // 0x40 - 0x80                             
byte engine_temp = 0x60;              // 0x60 - 0x90, 0xC0 - 0xCC, 0xB0 = overheat warning
byte gear_selection = 0x00;           // 0x00 = P, 0x20 = R, 0x40 = N, 0x60 = D, 0x80 = M, 0xC0 = 2, 0xA0 = 1
byte manual_gear = 0x10;              // 0x10 = 1, 0x20 = 2, 0x30 = 3
byte hydro1 = 0x40;                   // 0x40 = off, 0x80 = on
byte hydro2 = 0x00;                   // 0x00 = off, 0x20 = on


const byte seatbelt[8] = {0x80, 0x5F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x28};
const byte battery_light_off[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const byte door_ajar[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};             
const byte abs_data[8] = {0x00, 0x00, abs_light, 0x00, 0x00, 0x00, 0x00, stability_control};
const byte MIL_oil_pressure[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};                    //turns off MIL and oil pressure lights
const byte keepOn[8] = {0x40, turn_signal, backlight, 0x0A, 0x4C, 0x00, parking_brake_light, 0x00};      //wakeup message
const byte hydroboost1[8] = {0x00, 0x00, 0x00, hydro1, 0x00, 0x00, 0x00, 0x00}; 
const byte hydroboost2[8] = {0x83,0x69, 0x81, 0x4F, 0x81, 0x4F, hydro2, 0x00};

void loop()
{
  unsigned long currentMillis = millis();
  
  if(!digitalRead(CAN1_INT)) readCAN();                         // If CAN1_INT pin is low, read receive buffer
  if(keyOn){  
    if(currentMillis - previousMillis > interval){
      previousMillis = currentMillis;
      sendData();
      counter++;
    }
  }
}

void readCAN(){
  CAN1.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  //if(debug) printMessage(rxId, len, rxBuf);

  if(rxId == 0x3B2){                              //vehicle power state (first byte): 0x10 = off, 0x20 = ACC, 0x40 = on
    if(debug) printMessage(rxId, len, rxBuf);
    if(rxBuf[0] == 0x40) keyOn = true;
    else keyOn = false;
  }
  if(rxId == 0x3C3){                             //reading brake pedal (2nd byte): 0x00 = off, 0x01 = on 
    if(debug) printMessage(rxId, len, rxBuf);
    if(rxBuf[1] == 0x01) brake_pedal_pressed = true;
    else brake_pedal_pressed = false;
  }
  
}

void sendData(){
  byte sndStat = CAN0.sendMsgBuf(0x3B3, 0, 8, keepOn);
  if(sndStat != CAN_OK) Serial.println("Error Sending keepOn Message...");

  sndStat = CAN0.sendMsgBuf(0x3AE, 0, 8, door_ajar);              //cancel door ajar message and chime
  sndStat = CAN0.sendMsgBuf(0x42C, 0, 8, battery_light_off);      //turns off battery light

  sndStat = CAN0.sendMsgBuf(0x165, 0, 8, hydroboost1);
  //sndStat = CAN0.sendMsgBuf(0x200, 0, 8, hydroboost2);

  byte gear_data[8] = {manual_gear, 0x10, 0x00, 0x00, gear_selection, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x151, 0, 0, gear_data);              
  
  byte engine_temp_data[8] = {engine_temp, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x156, 0, 8, engine_temp_data);
  byte trans_temp_data[8] = {0x00, 0x00, 0x00, trans_temp, 0x00, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x230, 0, 8, trans_temp_data);
  if(sndStat != CAN_OK) Serial.println("Error Sending Temperature Message...");

  sndStat = CAN0.sendMsgBuf(0x415, 0, 8, abs_data);
  sndStat = CAN0.sendMsgBuf(0x420, 0, 8, MIL_oil_pressure);  

  byte rpm_speed_data[8] = {rpm, 0x4B, speed, 0x00, 0x00, 0x00, 0x00, 0x00};
  sndStat = CAN0.sendMsgBuf(0x201, 0, 8, rpm_speed_data);
  if(sndStat != CAN_OK) Serial.println("Error Sending RPM/Speed Message...");

  //GAUGE SWEEPING
  if(brake_pedal_pressed){ 
    speed++;
    if(speed > 0x40) speed = 0x00;
    rpm++;
    if(rpm > 0x60) rpm = 0x00;
  }
  engine_temp++;
  if(engine_temp > 0xAF) engine_temp = 0x60;
  trans_temp++;
  if(trans_temp > 0x80) trans_temp = 0x40;
  //Serial.println(engine_temp);
}

void printMessage(long unsigned int id, unsigned char length, unsigned char buffer[8]){
  sprintf(msgString, "ID: 0x%.3lX   Data:", id, length);

  Serial.print(msgString);

  for(byte i = 0; i < length; i++){
    sprintf(msgString, " 0x%.2X", buffer[i]);
    Serial.print(msgString);
  }
      
  Serial.println();
}
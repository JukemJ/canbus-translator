#include <mcp_can.h>
#include <SPI.h>

unsigned long previousMillis = 0;
long interval = 200;                        //sendData interval in milliseconds
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
bool keyOn = false;
bool debug = true;
bool brake_pedal_pressed = false;

#define CAN1_INT 2                              // Set INT to pin 2

//CAN0 is output to IPC
//CAN1 is input from BCM

MCP_CAN CAN0(9);      // Set CS to pin 9
MCP_CAN CAN1(8);      // Set CS to pin 8

void setup()
{
  Serial.begin(115200);
  delay(3000);          //wait for board startup

  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN0 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN0...");

  if(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN1...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  CAN1.setMode(MCP_NORMAL); 
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

byte seatbelt[8] = {0x80, 0x5F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x28};
byte door_ajar[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};             
byte battery_light_off[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte abs_data[8] = {0x00, 0x00, abs_light, 0x00, 0x00, 0x00, 0x00, stability_control};
byte MIL_oil_pressure[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};                    //turns off MIL and oil pressure lights
byte keepOn[8] = {0x40, turn_signal, backlight, 0x0A, 0x4C, 0x00, parking_brake_light, 0x00};      //wakeup message

void loop()
{
  unsigned long currentMillis = millis();
  
  if(!digitalRead(CAN1_INT)) readCAN();                         // If CAN0_INT pin is low, read receive buffer
  if(keyOn){  
    if(currentMillis - previousMillis > interval){
      previousMillis = currentMillis;
      sendData();
    }
  }
}

void readCAN(){
  CAN1.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  //if(debug) printMessage(rxId, len, rxBuf);

  if(rxId == 0x3B2){
    if(debug) printMessage(rxId, len, rxBuf);
    if(rxBuf[0] == 0x40) keyOn = true;
    else keyOn = false;
  }
  if(rxId == 0x3C3){
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
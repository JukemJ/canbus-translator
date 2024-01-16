#include <mcp_can.h>
#include <SPI.h>

//--------------------------------------------------- PROGRAM VARIABLES ----------------------------------------------------
unsigned long previousMillis = 0;
long interval = 200;                        //sendData interval in milliseconds
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
bool debug = true;                          // set to true to print out read messages in readCAN function
long int counter = 0;

#define CAN0_INT 8                        //set interrupt pins for reading CAN
#define CAN1_INT 2                             
#define CAN2_INT 4
#define CAN3_INT 6

                                          // Set CS pins
MCP_CAN CAN0(A0);                         // CAN0 is output to IPC
MCP_CAN CAN1(3);                          // CAN1 is input from BCM
MCP_CAN CAN2(5);                          // CAN2 is input from platform
MCP_CAN CAN3(7);                          // CAN3 is input from Transit
                                         
//--------------------------------------------------- VEHICLE VARIABLES --------------------------------------------
byte sndStat;
byte check_engine_light = 0x00;       // 0x00 = off, 0x01 = on, 0x02 = flashing
byte oil_pressure = 0x00;             // 0x00 = off, 0x04 = on also controls gauge
byte backlight = 0x10;                // 0x00 = off, 0x10 = on
byte parking_brake_light = 0xC0;      // 0x00 = off, 0xC0 = on
byte speed = 0x00;                    // speed from 0x00 - 0x40(64)
byte rpm = 0x00;                      // rpm from 0x00 - 0x60(96)
byte turn_signal = 0x48;              // 0x_5 = right, 0x_6 = left, 0x_7 both
byte abs_light = 0x00;                // 0x00 = off, 0x01 = solid, 0x02= fast blink, 0x03 = slow blink
byte stability_control = 0x00;        // 0x00 = off, 0x50 = solid, 0x90 = slow blink, 0xF0 = fast blink
byte trans_temp = 0x40;               // 0x40 - 0x80                             
byte engine_temp = 0x60;              // 0x60 - 0x90, 0xC0 - 0xCC, 0xB0 = overheat warning
byte gear_selection = 0x00;           // 0x00 = P, 0x20 = R, 0x40 = N, 0x60 = D, 0x80 = M, 0xC0 = 2, 0xA0 = 1
byte manual_gear = 0x10;              // 0x10 = 1, 0x20 = 2, 0x30 = 3
byte hydroboost_light = 0x00;         // 0x00 = off, 0x80 = on
byte power_status = 0x00;              // 0x10 = off, 0x20 = ACC, 0x40 = key on
byte battery_light = 0x00;            // 0x00 = off, 0x10 = on

unsigned int vehicle_speed_kph = 0;
unsigned int state_of_charge_percent = 0;
bool keyOn = false;
bool brake_pedal_pressed = false;
bool minor_fault = true;
bool major_fault = true;
bool high_voltage_active = false;
bool low_voltage_active = false;

//--------------------------------------------------------------- MAIN -----------------------------------------------------
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

  if(CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN2 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN2...");

  if(CAN3.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 CAN3 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 CAN3...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  CAN1.setMode(MCP_NORMAL);
  CAN2.setMode(MCP_NORMAL);
  CAN3.setMode(MCP_NORMAL);
}

void loop()
{
  unsigned long currentMillis = millis();
  
  if(!digitalRead(CAN1_INT)) readCAN1();                         // If CAN1_INT pin is low, read receive buffer
  if(!digitalRead(CAN2_INT)) readCAN2();
  //if(!digitalRead(CAN3_INT)) readCAN3();
  //sendData();


  if(currentMillis - previousMillis > interval){                 //send data to CAN0 every interval (200ms)
      previousMillis = currentMillis;
      sendData();
      counter++;
    }
  
}

// ------------------------------------------- FUNCTIONS -----------------------------------------------
void readCAN1(){
  CAN1.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  if(debug) printMessage(rxId, len, rxBuf);

  if(rxId == 0x3B2){                              //vehicle power state (byte[0]): 0x10 = off, 0x20 = ACC, 0x40 = on
    if(debug) printMessage(rxId, len, rxBuf);

    if(rxBuf[0] == 0x40) power_status = 0x40;
    else power_status = 0x40;
  }

  if(rxId == 0x3C3){                             //reading brake pedal (byte[1]): 0x00 = off, 0x01 = on 
    if(debug) printMessage(rxId, len, rxBuf);

    if(rxBuf[1] == 0x01) brake_pedal_pressed = true;
    else brake_pedal_pressed = false;
  } 
}

void readCAN2(){
  CAN2.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

  //if(debug && rxId > 0xFFF) printMessage(rxId, len, rxBuf);

  if(rxId == 0x98FFE23C){                              //Extended ID (0x18FFE23C): speed, soc, gear selection, fault
    if(debug) printMessage(rxId, len, rxBuf);
    vehicle_speed_kph = rxBuf[0];
    state_of_charge_percent = rxBuf[2] * 256  + rxBuf[1];       //SoC info on byte[2] and byte[1] (multiplied by 100 e.g. 10000 == 100.00%)
    //Serial.println(state_of_charge_percent);
    //---------------------------------------------------------------===
    if(high_voltage_active) speed = state_of_charge_percent / 100 * 0.64;
    else speed = 0;                                                                 //for now using speedo for state of charge percentage
    //-----------------------------------------------------------------------
    if(bitRead(rxBuf[4],1) == 1) minor_fault = true;
    else minor_fault = false;
    if(bitRead(rxBuf[4],2) == 1) major_fault = true;
    else major_fault = false;
  }

  if(rxId == 0x98FF7C3C){                                  //Extended ID (0x18FF7C3C): parking brake
    if(debug) printMessage(rxId, len, rxBuf);
    if(rxBuf[0] == 0x51) power_status = 0x20;
    else if(rxBuf[0] == 0x55 || rxBuf[0] == 0x5F) power_status = 0x40;
    if(bitRead(rxBuf[0],4) == 1) parking_brake_light = byte(0xC0);        //byte[0] is parking brake status: on = 0x5F, off = 0x4F
    else parking_brake_light = byte(0x00);
    if(bitRead(rxBuf[0],3) == 1) high_voltage_active = true;
    else high_voltage_active = false;
    if(bitRead(rxBuf[0],5) == 1) low_voltage_active = true;
    else low_voltage_active = false;
  }

  if(rxId == 0x80FF7B47){                                 //Extended ID (0x00FF7B47)
    //gear request on byte[0]: N = 0xFA, D = 0xF2, R = 0xEA, 0xDA = parking brake request
  }

  if(rxId == 0x98FFE43C){                                     //Extended ID (0x18FFE43C)
    if(bitRead(rxBuf[2],0) == 1) brake_pedal_pressed = true;  //brake pedal status of platform on third byte 0x01 = pedal pressed, 0x00 = pedal+
    else brake_pedal_pressed = false;
  }
}

void sendData(){
  //----------------------------------------------------------- CANBUS MESSAGES -----------------------------------------
  byte seatbelt[8] = {0x80, 0x5F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x28};
  byte battery_light_data[8] = {0x00, battery_light, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte door_ajar[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};             
  byte abs_data[8] = {0x00, 0x00, abs_light, 0x00, 0x00, 0x00, 0x00, stability_control};
  byte MIL_oil_pressure[8] = {0x00, check_engine_light, oil_pressure, 0x00, 0x00, 0x00, 0x00, 0x00};                    // MIL and oil pressure lights
  byte keepOn[8] = {power_status, turn_signal, backlight, 0x0A, 0x4C, 0x00, parking_brake_light, 0x00};                 // wakeup message
  byte hydroboost[8] = {0x00, 0x00, hydroboost_light, 0x00, 0x00, 0x00, 0x00, 0x00}; 
  byte engine_temp_data[8] = {engine_temp, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00};
  byte trans_temp_data[8] = {0x00, 0x00, 0x00, trans_temp, 0x00, 0x00, 0x00, 0x00};
  byte rpm_speed_data[8] = {rpm, 0x4B, speed, 0x00, 0x00, 0x00, 0x00, 0x00};

  if(major_fault) check_engine_light = 0x02;            // flashing MIL for major fault
  else if(minor_fault) check_engine_light = 0x01;       // solid MIL for minor fault
  else check_engine_light = 0x00;
  if(high_voltage_active) battery_light = 0x00;         // turn on battery light if HV fault
  else battery_light = 0x10;
  CAN0.sendMsgBuf(0x3B3, 0, 8, keepOn);
  CAN0.sendMsgBuf(0x3AE, 0, 8, door_ajar);              // cancel door ajar message and chime
  CAN0.sendMsgBuf(0x42C, 0, 8, battery_light_data);
  CAN0.sendMsgBuf(0x156, 0, 8, engine_temp_data);
  CAN0.sendMsgBuf(0x230, 0, 8, trans_temp_data);
  CAN0.sendMsgBuf(0x415, 0, 8, abs_data);
  CAN0.sendMsgBuf(0x40C, 0, 8, hydroboost);
  CAN0.sendMsgBuf(0x420, 0, 8, MIL_oil_pressure);  
  CAN0.sendMsgBuf(0x201, 0, 8, rpm_speed_data);

  // sndStat = CAN0.sendMsgBuf(0x165, 0, 8, hydroboost1);
  // sndStat = CAN0.sendMsgBuf(0x200, 0, 8, hydroboost2);

  // byte gear_data[8] = {manual_gear, 0x10, 0x00, 0x00, gear_selection, 0x00, 0x00, 0x00};
  // sndStat = CAN0.sendMsgBuf(0x151, 0, 0, gear_data);              

  //GAUGE SWEEPING
  if(brake_pedal_pressed){ 
    rpm = 40;
  
  }
  else rpm = 10;
  // engine_temp++;
  // if(engine_temp > 0xAF) engine_temp = 0x60;
  // trans_temp++;
  // if(trans_temp > 0x80) trans_temp = 0x40;
}
// -------------------------------------- PRINT TO SERIAL FOR DEBUGGING -----------------------------------------------
void printMessage(long unsigned int id, unsigned char length, unsigned char buffer[8]){
  sprintf(msgString, "ID: 0x%.3lX   Data:", id, length);

  Serial.print(msgString);

  for(byte i = 0; i < length; i++){
    sprintf(msgString, " 0x%.2X", buffer[i]);
    Serial.print(msgString);
  }
      
  Serial.println();
}
# CAN Translator
This is an Arduino based CANBUS translator for our prototype vehicle. This allows for different platforms to interface directly together. Specifically a 2019 F-650, a 2022 Transit, a 2023 F-650 and an experimental platform. 

# Wiring

| Arduino | MCP2515 | MCP2515 | 
| :--: | :---: | :---: |
| 5V | VCC | VCC |
| GND | GND | GND |
| 9 | CS |   |
| 8 |   | CS |
| 12 | SO | SO |
| 11 | SI | SI |
| 13 | SCK | SCK |
| 2  | INT | INT |

# Code
The code takes input on the CAN1 interface from the BCM and remaps the signals before sending them out on the CAN0 interface to the instrument cluster.

## Input

### CAN1
Below is a list of messages that are filtered out and recieved. All IDs are in hex format. Details about the specific data in messages is written in code comments.

| ID | Description |
| :---: | :--- |
| 3B2 | Contains power mode info. Used to wake up the IPC when IGN is in run |
| 3C3 | Second byte indicates brake pedal switch status |

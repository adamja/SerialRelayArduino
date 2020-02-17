/*
 * Communication bits:
+----------+---------------+---------------------------------------------------------------------+
| Function | Decimal Value |                             Description                             |
+----------+---------------+---------------------------------------------------------------------+
| STX      |             2 | Start of text (used before each command message sent/received)      |
| ETX      |             3 | End of text (used at the end of each command message sent/received) |
+----------+---------------+---------------------------------------------------------------------+

 * Receive commands:
+-------+----------+---------+
| Relay | Function | Command |
+-------+----------+---------+
| All   | ON       | RA-ON   |
| All   | OFF      | RA-OFF  |
| 1     | ON       | R1-ON   |
| 1     | OFF      | R1-OFF  |
| 2     | ON       | R2-ON   |
| 2     | OFF      | R2-OFF  |
| 3     | ON       | R3-ON   |
| 3     | OFF      | R3-OFF  |
| 4     | ON       | R4-ON   |
| 4     | OFF      | R4-OFF  |
+-------+----------+---------+

 * Reply commands:
+------------+---------------------------------------------------------------------------------------------------------+
|  Command   |                                               Description                                               |
+------------+---------------------------------------------------------------------------------------------------------+
| HEARTBEAT  | Sends a heartbeat message at regular intervals to indicate arduino is still online                      |
| INVALID    | Sends an invalid message in reply to a received message if it does not match a known command            |
| RESET      | Sends a reset command when the arduino sketch starts running                                            |
| SAFETY-OFF | Sends a safety off message to indicate that a relay has been left on for longer than                    |
|            | the timeout duration and the arduino has automatically switched all relays off even though no command   |
|            | was received via serial                                                                                 |
| RX-ON      | Sends a relay X on command to indicate that the command was received and the relay has been turned on   |
| RX-OFF     | Sends a relay X off command to indicate that the command was received and the relay has been turned off |
+------------+---------------------------------------------------------------------------------------------------------+
 */

#include <Arduino.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <QueueArray.h>

// Relays
#define relay1 4
#define relay2 5
#define relay3 6
#define relay4 7
#define RelayOn HIGH
#define RelayOff LOW

// HC-12
#define HC12_TX 2
#define HC12_RX 3
#define HC12_SET 4
SoftwareSerial HCSerial(HC12_TX, HC12_RX); //RX, TX

// Heartbeart
unsigned long heartbeat_delay = 60000;      // Send keepalive heartbeat message at this interval (ms)
unsigned long heartbeat_prev_millis = 0;
unsigned long heartbeat_count = 0;

// Safety timeout
unsigned long safetyTimeoutDelay = 7200000;       // 2 hours
unsigned long lastTurnedRelayOnMillis = 0;

// Watch dog timer
#define WDT_TIMEOUT WDTO_4S         // Watch dog timer delay time set to 4s

// Messaging
const char STX = 2;
const char ETX = 3;
byte incomingByte;
String readBuffer = "";
QueueArray <String> commandQueue;

// Functions
void setup();
void loop();
void processCommands();
void getCommand();
void sendHeartbeat();
void sendCommand(String command);
bool relayActive();
void safetyCheck();


void setup() {
  Serial.begin(9600);
  HCSerial.begin(9600);

  commandQueue.setPrinter (Serial);  // set the serial printer of the queue

  pinMode(HC12_SET, OUTPUT);
  digitalWrite(HC12_SET, LOW);  // Put the HC-12 in configuration mode
  
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  digitalWrite(relay1, RelayOff);
  digitalWrite(relay2, RelayOff);
  digitalWrite(relay3, RelayOff);
  digitalWrite(relay4, RelayOff);

  wdt_enable(WDTO_8S);  // Set a watchdog timer for 8 seconds

  sendCommand("RESET");
  sendCommand("HEARTBEAT");
}


void loop() {
  // ==== Storing the incoming data into a String variable
  while (Serial.available()) {  // If HC-12 has data
    incomingByte = Serial.read();  // Store each icoming byte from HC-12
    readBuffer += char(incomingByte);  // Add each byte to ReadBuffer string variable
  }
  delay(100);
  
  getCommand();
  processCommands();
  sendHeartbeat();
  if (relayActive()) {
    safetyCheck();
  }

  readBuffer = "";  // Clear readBuffer
  wdt_reset();  // Reset the watchdog timer
}

void getCommand() {
  
  boolean start = false;
  String command = "";

  for (int i = 0; i < readBuffer.length(); i++) {
    
    char c = readBuffer[i];
    
    // Reads the data between the start STX and end marker ETX
    if (start == true) {
      if (c != ETX) {
        command += c;    // Add each byte to command string variable
      }
      else {
        commandQueue.push(command);
        Serial.println("Received command: " + command);
        command = "";
        start = false;
      }
    }
    // Checks whether the received message statrs with the start marker STX
    else if (c == STX) {
      start = true; // If true start reading the message
    }
  }
}


void processCommands() {
  while (!commandQueue.isEmpty()) {
    String command = commandQueue.pop();

    if (command == "RA-ON") {
      digitalWrite(relay1, RelayOn);
      digitalWrite(relay2, RelayOn);
      digitalWrite(relay3, RelayOn);
      digitalWrite(relay4, RelayOn);
      sendCommand(command);
      lastTurnedRelayOnMillis = millis();
      
    } else if (command == "RA-OFF") {
      digitalWrite(relay1, RelayOff);
      digitalWrite(relay2, RelayOff);
      digitalWrite(relay3, RelayOff);
      digitalWrite(relay4, RelayOff);
      sendCommand(command);
      
    } else if (command == "R1-ON") {
      digitalWrite(relay1, RelayOn);
      sendCommand(command);
      lastTurnedRelayOnMillis = millis();
      
    } else if (command == "R1-OFF") {
      digitalWrite(relay1, RelayOff);
      sendCommand(command);
      
    } else if (command == "R2-ON") {
      digitalWrite(relay2, RelayOn);
      sendCommand(command);
      lastTurnedRelayOnMillis = millis();
      
    } else if (command == "R2-OFF") {
      digitalWrite(relay2, RelayOff);
      sendCommand(command);
      
    } else if (command == "R3-ON") {
      digitalWrite(relay3, RelayOn);
      sendCommand(command);
      lastTurnedRelayOnMillis = millis();
      
    } else if (command == "R3-OFF") {
      digitalWrite(relay3, RelayOff);
      sendCommand(command);
      
    } else if (command == "R4-ON") {
      digitalWrite(relay4, RelayOn);
      sendCommand(command);
      lastTurnedRelayOnMillis = millis();
      
    } else if (command == "R4-OFF") {
      digitalWrite(relay4, RelayOff);
      sendCommand(command);
      
    } else {
      sendCommand("INVALID");
    }
    delay(100);
  }
}


void sendHeartbeat() {
    unsigned long cur_millis = millis();
    if (heartbeat_prev_millis < cur_millis - heartbeat_delay && cur_millis > heartbeat_delay) {
        heartbeat_count++;
        sendCommand("HEARTBEAT");
        // sendCommand("HEARTBEAT-" + String(heartbeat_count));
        heartbeat_prev_millis = cur_millis;
    }
}

void sendCommand(String command) {
  HCSerial.print(STX + command + ETX);
  Serial.println("Sent command to HC-12: " + command);
}


bool relayActive() {
  if (digitalRead(relay1) == RelayOn) {
    return true;
  }
  if (digitalRead(relay2) == RelayOn) {
    return true;
  }
  if (digitalRead(relay3) == RelayOn) {
    return true;
  }
  if (digitalRead(relay4) == RelayOn) {
    return true;
  }
  return false;
}

void safetyCheck() {
  unsigned long cur_millis = millis();
  if (lastTurnedRelayOnMillis < cur_millis - safetyTimeoutDelay && cur_millis > safetyTimeoutDelay) {
    digitalWrite(relay1, RelayOff);
    digitalWrite(relay2, RelayOff);
    digitalWrite(relay3, RelayOff);
    digitalWrite(relay4, RelayOff);
    sendCommand("SAFETY-OFF");
  }
}

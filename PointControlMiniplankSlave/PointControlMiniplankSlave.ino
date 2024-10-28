#include <LibPrintf.h>
#include <Wire.h>
#include <CMRI.h>
#include <Auto485.h>
const int SENSOR_PIN[] = { 3, 4, 5, 6, A2, A3, A6, A7 }; // From sensors also connect A6 & A7 to JMRI MEGA pins 37 & 39
const int OUTPUT_PIN[] = { 7, 8, 9, 10, 11, 12 };// // to JMRI MEGA pins 25,27,29,31,33,35
const int POINTIN_PIN[] = {14, 15 }; // from JMRI MEGA pins 41 & 43
const int POINTOUT_PIN[] = {13,2}; // to JMRI MEGA pins 45 & 47



bool BlockNo[4];
bool SpotNo[4];
bool pointNo[2];
// unsigned long flashTime = 0;
// const int FLASH_DLY 8;

uint16_t sensorData = 1;   // Example 16-bit sensor data (representing 16 sensors)
uint16_t pointData = 0;    // Example 16-bit point data (representing 16 points sent from JMRI)
uint16_t swStatus = 0;     // swStatus received from the master
uint16_t osensorData = 0;  // old for printout
uint16_t opointData = 0;
uint16_t oswStatus = 0;

byte slaveAddress = 8;  // for i2c
unsigned long timeStamp = 0;
//const int CMRI_ADDRESS = 1;  // node address
//const uint8_t DE_PIN = 2;    //Pin for RS485 DE & RE connected together

//Auto485 bus(DE_PIN);                   // Arduino pin 2 -> MAX485 DE and RE pins together
//CMRI cmri(CMRI_ADDRESS, 32, 48, bus);  // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

// void sendComsTest() {
//   static int j = 0;
//   if (millis() > timeStamp + 500) {
//     timeStamp = millis();
//     sensorData = (sensorData << 1);
//     if (sensorData == 0) sensorData++;
//     if (sensorData > 0b1000000) sensorData = 0;
//     digitalWrite(OUTPUT_PIN[j], 1);
//     if (j > 0) {
//       digitalWrite(OUTPUT_PIN[j - 1], 0);
//     }
//     j++;
//     if (j > 8) {
//       digitalWrite(OUTPUT_PIN[j - 1], 0);
//       //digitalWrite(OUTPUT_PIN[0],1);
//       j = 0;
//     }
//   }
// }
// bool flashIR() {
//   if (millis() - flashTime >= FLASH_DLY) {
//     IRon = !IRon;
//     digitalWrite(PIN_IR, IRon);
//   }
//   bool IRSensor = digitalRead(PIN_IR);
//   if (!IRon) IREnabled = !IRSensor;
//   return (IRon && IRSensor && IREnabled)
// }

// bool getSensorValue(int i) {
//   return !digitalRead(SENSOR_PIN[i]);
// }
void printBits(long int n) {
  byte numBits = 16;  // 2^numBits must be big enough to include the number n
  char b;
  char c = ' ';  // delimiter character
  for (byte i = 0; i < numBits; i++) {
    // shift 1 and mask to identify each bit value
    b = (n & (1 << (numBits - 1 - i))) > 0 ? '1' : '0';  // slightly faster to print chars than ints (saves conversion)
    Serial.print(b);
    if (i < (numBits - 1) && ((numBits - i - 1) % 4 == 0)) Serial.print(c);  // print a separator at every 4 bits
  }
}
// void getSetData() {
//   // get the JMRI point setting
//   // set the switch setting
//   // set the sensor settings
//   //cmri.process();                           // get JMRI data via CMRI bits
//   for (int i = 0; i < 16; i++) {        // just using 16 outputs
//     pointData |= cmri.get_bit(i) << i;  // get new incoming status for point positions
//     cmri.set_bit(i, bitRead(swStatus, i));
//     cmri.set_bit(i, bitRead(sensorData, i));
//   }
// }
void debugPrint() {
  if (sensorData != osensorData || pointData != opointData || swStatus != oswStatus) {

    Serial.print("\nSensors incoming from layout to Panel and JMRI: ");
    printBits(sensorData);
    Serial.print("\nIncoming Points from JMRI forwarded to panel  : ");
    printBits(pointData);
    Serial.print("\nSwitches incoming from panel forwarded to JMRI: ");
    printBits(swStatus);

    Serial.println("");
    osensorData = sensorData;
    opointData = pointData;
    oswStatus = swStatus;
  }
}

void setup() {
  Wire.begin(slaveAddress);      // Join I2C bus with address #8 (slave address)
  Wire.onRequest(requestEvent);  // Register function to run when data is requested by master
  Wire.onReceive(receiveEvent);  // Register function to run when data is received from master
  Serial.begin(19200);
  printf("I2C link to sensors on MiniPlank \n Steve Lomax Not for commercial use \n\"PointControlMiniplankSlave\" 03/10/24");
  for (int i = 0; i < 8; i++) {
    pinMode(SENSOR_PIN[i], INPUT_PULLUP);
    if (i < 6) {
      pinMode(OUTPUT_PIN[i], OUTPUT);
    }
  }
  pinMode(POINTIN_PIN[0], INPUT_PULLUP);
  pinMode(POINTIN_PIN[1], INPUT_PULLUP);
  pinMode(POINTOUT_PIN[0], OUTPUT);
  pinMode(POINTOUT_PIN[1], OUTPUT);
}

void readSensors() {
  uint16_t tData = 0;
  for (int i = 0; i < 8; i++) {
    bool sensorVal = 0;
    if (i < 6) {
      sensorVal = !digitalRead(SENSOR_PIN[i]);  //READS 6 DIGITAL PINS
    } else {
      if (analogRead(i) < 100) sensorVal = 1;  //reads 2 analog pins 7 & 8 which cant be digital
    }
    if (i < 6) {
      digitalWrite(OUTPUT_PIN[i], sensorVal);  //output sensors 1-6. sensors 7&8 hardwired
    }
    //printf("%d :%d  ", i, sensorVal);
    tData |= sensorVal << i;  // addend tData with all 8 sensors
    //printf("%d ", sensorVal);
  }
  //printf("\n");
  sensorData = tData;  //update sensorValue in one go to avoid partial data being sent
  //printf("\n");
}
void sendSwitchestoJMRI() {
  digitalWrite(POINTOUT_PIN[0], swStatus & 0b1);
  digitalWrite(POINTOUT_PIN[1], swStatus & 0b10);
  if (oswStatus != swStatus) printf("confirm writing to POINTOUT_PINs [0]: %d and [1]: %d\n", swStatus & 0b1,swStatus & 0b10);
}
void readjmripoints() {
  pointData = 0;
  int swVal = !digitalRead(POINTIN_PIN[0]);
  //printf("\na0 =   %d\n", !swVal);
  // printf("\na0 = not  %d\n", swVal);
  pointData |= swVal;
  swVal = !digitalRead(POINTIN_PIN[1]);
  pointData |= swVal << 1;
    if (opointData != opointData) printf("confirm reading from  POINTin_PINs [0]: %d and [1]: %d\n", pointData & 0b1,pointData & 0b10);

}
void loop() {
  // digitalWrite(13,0);
  // digitalWrite(2,0);
  // delay (500);
  // digitalWrite(13,1);
  // digitalWrite(2,1);
  // delay (500);
  readSensors();
  readjmripoints();
  sendSwitchestoJMRI();
  debugPrint();
}

// Function that executes whenever the master requests data
void requestEvent() {
  // Send sensorData and pointData as 16-bit integers to the master

  Wire.write(highByte(sensorData));  // Send high byte of sensorData
  Wire.write(lowByte(sensorData));   // Send low byte of sensorData
  Wire.write(highByte(pointData));  // Send high byte of jmri pointData
  Wire.write(lowByte(pointData));   // Send low byte of jmri pointData
  

  // Debug output

  // Serial.print("\nSent sensor data : ");
  // for (int k = 0; k < 8; k++) {
  //   bool bt = (sensorData >> k) & 1;
  //   Serial.print(bt);
  // }
  // Serial.print(" as sent : ");
  // Serial.print(sensorData, BIN);
  // Serial.print(", point data : ");
  // Serial.println(pointData, BIN);
}

// Function that executes whenever data is received from the master
void receiveEvent(int howMany) {
  if (howMany == 2) {             // Expecting 2 bytes for swStatus
    swStatus = Wire.read() << 8;  // Read the high byte of swStatus
    swStatus |= Wire.read();      // Read the low byte of swStatus

    // Debug output
  }
}

#include <LibPrintf.h>
#include <Wire.h>
const int SENSOR_PIN[] = {3,4,5,6,A2,A3,A6,A7};
const int OUTPUT_PIN[] = {7,8,9,10,11,12,13,2};
const int POINTOUT_PIN[] = {A0,A1};

bool BlockNo[4];
bool SpotNo[4];
bool pointNo[2];
unsigned long flashTime = 0;
const int FLASH_DLY 8;

uint16_t sensorData =1 ;  // Example 16-bit sensor data (representing 16 sensors)
uint16_t pointData =0 ;   // Example 16-bit point data (representing 16 points)
uint16_t swStatus = 0;        // swStatus received from the master
byte slaveAddress = 8;
unsigned long timeStamp=0;

void sendComsTest(){
  static int j=0;
  if (millis()>timeStamp+500){
    timeStamp = millis();
    sensorData=(sensorData<<1);
    if (sensorData==0) sensorData++;
    if (sensorData>0b1000000) sensorData= 0;
    digitalWrite(OUTPUT_PIN[j],1);
    if (j>0 ){
      digitalWrite(OUTPUT_PIN[j-1],0);
    }
    j++;
    if(j>8){
      digitalWrite(OUTPUT_PIN[j-1],0);                                                            
      //digitalWrite(OUTPUT_PIN[0],1);
      j=0;
    }
  }
}
bool flashIR() {
  if (millis() - flashTime >= FLASH_DLY) {
    IRon = !IRon;
    digitalWrite(PIN_IR, IRon);
  }
  bool IRSensor = digitalRead(PIN_IR);
  if (!IRon) IREnabled = !IRSensor;
  return (IRon && IRSensor && IREnabled)
}
bool getSensorValue(int i) {
  if (i < 4) return !digitalRead(PIN_SENSOR[i]);
  return flashIR() 
 
}

void setup() {
  Wire.begin(slaveAddress);                 // Join I2C bus with address #8 (slave address)
  Wire.onRequest(requestEvent);  // Register function to run when data is requested by master
  Wire.onReceive(receiveEvent);  // Register function to run when data is received from master
  Serial.begin(19200);
  printf("I2C link to sensors on MiniPlank");
  for(int i = 0;i<8;i++){
    pinMode (SENSOR_PIN[i], INPUT_PULLUP);
    pinMode (OUTPUT_PIN[i],OUTPUT);
  }
  pinMode (POINTOUT_PIN[0],OUTPUT);
  pinMode (POINTOUT_PIN[1],OUTPUT);
}


void loop() {
//sendComsTest();
for (int i = 0; i < 8; i++) {
    bool sensorVal = getSensorValue(i);
    digitalWrite(OUTPUT_PIN[i], sensorVal);
    printf("%d ", sensorVal);
  }
  printf("\n");
  
  
}

// Function that executes whenever the master requests data
void requestEvent() {
  // Send sensorData and pointData as 16-bit integers to the master
  Wire.write(highByte(sensorData));  // Send high byte of sensorData
  Wire.write(lowByte(sensorData));   // Send low byte of sensorData

  Wire.write(highByte(pointData));  // Send high byte of pointData
  Wire.write(lowByte(pointData));   // Send low byte of pointData

  // Debug output
  Serial.print("Sent to master: ");
  Serial.print(sensorData, BIN);
  Serial.print(", ");
  Serial.println(pointData, BIN);
}

// Function that executes whenever data is received from the master
void receiveEvent(int howMany) {
  if (howMany == 2) {             // Expecting 2 bytes for swStatus
    swStatus = Wire.read() << 8;  // Read the high byte of swStatus
    swStatus |= Wire.read();      // Read the low byte of swStatus

    // Debug output                                                                     
    Serial.print("Received swStatus from master: ");
    Serial.println(swStatus, BIN);
  }
}
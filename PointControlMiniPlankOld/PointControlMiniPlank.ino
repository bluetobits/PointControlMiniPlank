
//Git version 24/09/24
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastLED.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <LibPrintf.h>
#include <CMRI.h>
#include <Auto485.h>
#include <avr/wdt.h>
// pointPairing The master point of a point pair must be the point with the lower point number
// the pointPairs position is the actual point, the number is the point that controls it.
// multiple points can be controlled by 1 point, a negative number indicated the slave point is inverter.
/*pressing the encoder enters and exits different functions:
1 long press > 3s = ENTER CALIBRATION. 
    change switch position to change point under calibration
    move encoder to adjust position
    1 long press saves all new positions and exit calibration
    1 short press undo changes and exit calibration
2 short presses in 3s Toggles POINT SLAVE GROUPING on/off
3 short presses in 3s Toggles CENTRE SERVO POSITION
    move a switch to centre that servo
4 short presses in 3s SET MOVE SPEED or hold in encoder while resetting
    adjust encoder to increase or decrease servo speed
    move a point to view the current speed setting
    1 long press saves new move speed and exit
    1 short press undo changes and exit 
5 short presses in 3s - reset switch panel & reboot in 2 seconds

indicators:
POINTS
GREEN  = This point branch selected
RED = This point brance deselected
CYAN = This slave point (grouped) branch is selected
ORANGE = This slave point (grouped) branch is deselected
BLUE FLASHING = this point is moving
ALTERNATING FLASHING = under auto control
PURPLE = this point setting is under calibration
YELLOW FLASHING = servo in centre position

SENSORS
PINK = track block occupied
YELLOW = stop sensor activated
ORANGE FLASHING = lost communication with sensor module / error 








*/
// SWITCHING DATA CHANGE ALL CAPITALISED VALUES TO SUIT
const uint8_t NO_OF_POINTS = 2;         // no of szwitches / points on the switch panel
const uint8_t ROW_PINS[] = { 6 };       //Arduino pin numbers for output switch Matrix
const uint8_t NO_OF_ROWS = 1;           //the number of rows the switches are matrix wired (see schematic)
const uint8_t COL_PINS[] = { A1, A0 };  //Arduino pin numbers for analog input switch Matrix
const uint8_t NO_OF_COLS = 2;           //the number of columns the switches are matrix wired (see schematic)
const int8_t POINT_PAIRS[] = { 0, 0 };  // change the number in point number position. read notes above.
const int TOP_PULSE_LEN = 2400;         // setting the maximum cw servo position(actual = 2500 but not all servos are the same)
const int BOTTOM_PULSE_LEN = 600;       //setting the minimum ccw servo position
bool pointPairing = 0;                  // Global pairing slave points with master points
const uint8_t NO_OF_LEDS = 12;
const uint8_t NO_OF_BLOCKS = 4;
const uint8_t NO_OF_SENS = 4;
// total number of WS2812b LEDS
const uint8_t LEDS_MIMIC[] = { 2, 6, 7, 8, 1, 3, 9, 5, 0, 4, 10, 11 };  // the order of the leds. point 0 Closed, Point 0 Thrown, Point 1 closed, 1 thrown, etc the rest are sensors

// end of user settings

int moveSpeed = 4;           //Global. point move speed
bool moving = 0;             //Global. at least one  pooint is moving
bool cal = 0;                //Global. calibrating  status
uint16_t swStatus = 0;       //Global. 16 bits showing switch status
uint16_t oldswStatus = 0;    //Global. records 16 bits for the previous loop switch status
uint8_t lastPointMoved = 0;  //Global.  for calibrating. closed positions are 0-15 and thrown positions are 16-31




// translate switch order to mimic led order 0closed, 0thrown, 1closed, 1thrown, 2closed, 2thrown...
unsigned long timeNow;
unsigned long flashTimeNow;
unsigned long flashTimeout = 0;        //Global.  Neopixel flash timer
bool flash = 0;                        //Global.  Neopixel flash status
const unsigned long flashSpeed = 400;  // flash rate in ms
uint8_t onSat = 255;                   // Global. Neopixel saturation level (0-255)
uint8_t onLev = 100;                   // Global. Neopixel brightness level (0-255)
uint8_t onHue = 255;                   // Global. Neopixel colour hue level (0-255)


const uint8_t EEPROM_ADDRESS = 0;  // Starting address in EEPROM
bool onceflag = 0;                 // debugging for single printout only
//encoder
const uint8_t ENCA_PIN = 3;      // encoder pin A
const uint8_t ENCB_PIN = 4;      // encoder pin B
const uint8_t ENCODER_PUSH = 5;  // push button pin
int32_t encoderPos = 0;          //Global.  current/previous encoder position
int oldLastPointMoved = 0;       // Global. previuous point calibrated

//JMRI  and RS485 connections
const int CMRI_ADDRESS = 0;  // node address
const uint8_t DE_PIN = 2;    //Pin for RS485 DE & RE connected together
int incoming = 0;
byte incomingSensors;
// bits from JMRI data
byte slaveAddress = 8;
byte i2cError = 255;

// panel HID
const uint8_t MOV_LED = 13;    // LED to denote points are moving
const uint8_t CAL_LED = 11;    // LED to denote in calibration mode
const uint8_t DATA_PIN = 10;   // neopixels data connect from here via 50 ohm resistor
const int LONG_PUSH = 3000;    // the "long push" duration of encoder button
bool changeMoveSpeed = false;  // flag to idnicate push is held in from startup
bool centreServo = false;      // flag to indicate servo centre mode
//Initialisation
CRGB leds[NO_OF_LEDS];  //LED neopixel strip
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
Encoder encoder(ENCA_PIN, ENCB_PIN);
Auto485 bus(DE_PIN);                   // Arduino pin 2 -> MAX485 DE and RE pins together
CMRI cmri(CMRI_ADDRESS, 24, 48, bus);  // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs



struct MemStruct {
  int mclosedPos[16];
  int mthrownPos[16];
  int mmoveSpeedMem;
  bool pointPairing;  // save point pairing enabled status

  void readEEPROM() {
    EEPROM.get(EEPROM_ADDRESS, *this);
  }
  void writeEEPROM() {
    EEPROM.put(EEPROM_ADDRESS, *this);
  }
};

MemStruct memData;


struct Points {  // each point has these properties and method
  int closedPos = 1000;
  int thrownPos = 2000;
  int curPos = 1000;
  bool swPos = 1;
  bool autoControl = 0;  // from JMRI

  void movePoint(int index) {

    //moving = 0;
    if (swPos) {  // if target is thrown
      if (curPos != thrownPos) {
        increment(thrownPos, index);
      }
    } else {
      if (curPos != closedPos) {
        increment(closedPos, index);
      }
    }
  }
  void increment(int newPos, int index) {

    int totDist = thrownPos - closedPos;
    totDist = abs(totDist);
    int dtg = curPos - newPos;
    int incMove = totDist / 300 * moveSpeed;
    if (abs(dtg) <= incMove) {  // if within MOVE_STEP
      curPos = newPos;
      printf("moved %d distance %d at speed %d\n ", index, totDist, incMove);
    } else if (curPos < newPos) {
      curPos += incMove;
    } else {
      curPos -= incMove;
    }
    servo.writeMicroseconds(index, curPos);  //move servo
    moving = 1;
  }
};
// void movePoint(int index) {
//   int dtg;
//   int totDist;
//   //moving = 0;
//   if (swPos) {  // if target is thrown
//     if (curPos != thrownPos) {
//       //if point is not at target not at target
//       totDist = thrownPos - closedPos;
//       totDist = abs(totDist);
//       dtg = curPos - thrownPos;
//       if (abs(dtg) <= moveSpeed) {  // if within MOVE_STEP
//         curPos = thrownPos;
//         printf("thrown %d distance %d at speed %d\n ", index, totDist,moveSpeed);
//       } else if (curPos < thrownPos) {
//         curPos += moveSpeed;
//       } else {
//         curPos -= moveSpeed;
//       }
//       servo.writeMicroseconds(index, curPos);  //move servo
//       moving = 1;
//     }
//   } else {                      // target is closed
//     if (curPos != closedPos) {  // if not yet at target
//     totDist = thrownPos - closedPos;
//       totDist = abs(totDist);
//       dtg = curPos - closedPos;
//       if (abs(dtg) <= moveSpeed) {
//         curPos = closedPos;
//         printf("closed %d Distance %d at speed %d\n ", index, totDist,moveSpeed);
//       } else if (curPos < closedPos) {
//         curPos += moveSpeed;
//       } else {
//         curPos -= moveSpeed;
//       }
//       servo.writeMicroseconds(index, curPos);  //move servo
//       moving = 1;
//     }
//   }
// }


Points point[NO_OF_POINTS];

void savePointValues() {
  MemStruct memData;
  for (int i = 0; i < NO_OF_POINTS; i++) {
    memData.mthrownPos[i] = point[i].thrownPos;
    memData.mclosedPos[i] = point[i].closedPos;
    memData.mmoveSpeedMem = moveSpeed;
    memData.pointPairing = pointPairing;
  }
  memData.writeEEPROM();
}
void loadPointValues() {
  MemStruct memData;
  memData.readEEPROM();
  for (int i = 0; i < NO_OF_POINTS; i++) {  // transfer memory values to point values
    point[i].thrownPos = memData.mthrownPos[i];
    point[i].closedPos = memData.mclosedPos[i];
    moveSpeed = memData.mmoveSpeedMem;
    pointPairing = memData.pointPairing;
  }
}

void printTimeVars() {
  Serial.print("   millis = ");
  Serial.print(millis());
  Serial.print(".   timeNow = ");
  Serial.print(timeNow);
  Serial.print(".   flashTimeNow = ");
  Serial.print(flashTimeNow);
  Serial.print(".   FlashTimeout = ");
  Serial.println(flashTimeout);
}


//============================== READ SWITCH POSITIONS =================
void readSwitches() {
  oldswStatus = swStatus;
  swStatus = 0;  // Clear previous value

  for (int row = 0; row < NO_OF_ROWS; row++) {    // iterate through all 4 switch rows.
    digitalWrite(ROW_PINS[row], HIGH);            // set the current row high
    for (int col = 0; col < NO_OF_COLS; col++) {  //iterate through all 4 switch cols.
      bool curPointStatus;                        // temp store the current point switch pos before reading in the new status
      int i = (row * NO_OF_ROWS) + col;           // set i for each switch counting across rows for each columns
      curPointStatus = point[i].swPos;            // temp save old switch position to see if it will change for calibrating
      int swIn = analogRead(COL_PINS[col]);       // read the value of each col


      if (!cal && POINT_PAIRS[i] != i && pointPairing) {  //if a slave and not calibrating and pointpairing is enabled
        point[i].swPos = point[abs(POINT_PAIRS[i])].swPos;
        if (POINT_PAIRS[i] < 0) point[i].swPos = !point[i].swPos;  // match master point setting
        //printf ("point number %d  is paired with = %d and is set to %d\n",i,pointPairs[i],point[i].swPos );

      } else {


        point[i].autoControl = 0;
        if (swIn < 300) {  // switch close to ground and therefore thrown
          //printf("i = %d, swIn = %d\n", i, swIn);
          point[i].swPos = true;
        } else if (swIn >= 300 && swIn < 600) {  //switch in mid position therefore closed
          point[i].swPos = 0;
        } else if (swIn > 600) {                 // switch is opposite thrown and therefore under computer control
          point[i].swPos = (incoming >> i) & 1;  // set to incoming CMRI/JMRI position
          point[i].autoControl = 1;
        }
        // if (swIn < 10) printf(" ");
        // if (swIn < 100) printf(" ");
        // printf("%d %d ", swIn, point[i].curPos);
      }
      if (curPointStatus != point[i].swPos) {                // if it has changed
        lastPointMoved = i;                                  // note number of last point moved
        if (point[i].swPos) lastPointMoved += NO_OF_POINTS;  // if point is thrown then add (NO OF POINTS) to the point number
        delay(100);                                          //switch de-bounce
        //printf(" row %d, Col %d, index %d, value %d\n ",row,col,i,swIn);
      }
      //printf("%d at row %d, col %d\n",point[i].swPos,row,col);
      swStatus = swStatus + (point[i].swPos << i);  // convienient place to generate the
    }
    digitalWrite(ROW_PINS[row], LOW);
  }
  // printf("\n");
}

//======================== CHANGE MOVE SPEED ==============================
void pointMoveSpeed() {
  encoderPos = encoder.read();

  digitalWrite(CAL_LED, flash);
  if (encoderPos > 3) {
    moveSpeed++;
    printf("move speed rising to %d\n", moveSpeed);
    encoder.write(0);
  } else if (encoderPos < -3) {
    if (moveSpeed > 1) moveSpeed--;
    printf("move speed falling to %d\n", moveSpeed);
    encoder.write(0);
  }

  if (digitalRead(ENCODER_PUSH) == 0) {
    unsigned long timepressed = millis();
    bool longPress = 0;
    while (digitalRead(ENCODER_PUSH) == 0) {
      //if (timeNow > timepressed + LONG_PUSH) {
      if (timepressed + LONG_PUSH < millis()) {
        longPress = 1;
        digitalWrite(CAL_LED, 0);
      }
      wdt_reset();  // Reset watchdog to prevent reset
    }
    if (longPress) {
      savePointValues();
    } else {
      loadPointValues();
    }
    changeMoveSpeed = false;
    digitalWrite(CAL_LED, 1);
  }
}

//----------------------- PRINT DATA FOR DEBUGGING -------------------
void printOutData() {
  for (int i = 0; i < NO_OF_POINTS; i++) {
    if (i < 10) {
      printf("  P%d ", i);
    } else {
      printf(" P%d", i);
    }
    if (point[i].curPos != point[i].closedPos && point[i].swPos == 0) {
      printf("   point %d is at %d closing to %d \n", i, point[i].curPos, point[i].closedPos);
    } else if (point[i].curPos != point[i].thrownPos && point[i].swPos == 1) {
      printf("   point %d is at %d throwing to %d \n", i, point[i].curPos, point[i].thrownPos);
    }
  }

  if (moving) {
    for (int i = 0; i < NO_OF_POINTS; i++) {
      printf("  %d  ", point[i].swPos);
    }
    //int temp = lastPointMoved / 2 + lastPointMoved % 2;
    printf(" last point moved = %d ", lastPointMoved);
    printf("\n");
  }
}

//=========================== GET & SET DATA FROM JMRI/CMRI==================
/*
connect to 2nd "Slave" Arduino over I2C
request 16 bits of sensor data
send 16 bits of point data

s
*/
void getSetData() {
  int oldincoming = incoming;
  incoming = 0;
  cmri.process();                           // get JMRI data via CMRI bits
  for (int i = 0; i < NO_OF_POINTS; i++) {  // just using 16 outputs
    incoming |= cmri.get_bit(i) << i;       // get new incoming status for point positions
    cmri.set_bit(i, point[i].swPos);        //set CMRI bits
  }
  if (oldincoming != incoming) {

    Serial.print("\nincoming = ");
    Serial.println(incoming, BIN);
    Serial.print("outgoing = ");
    Serial.println(swStatus, BIN);
  }
}

// ============================ SET LEDS ==============================
void setLeds() {
  flashTimeNow = millis();
  if (flashTimeNow > (flashTimeout)) {  // every 400ms change flash on to flash off or vice versa

    flash = !flash;  // whatever flash state is (on or off), make it the opposite
    //digitalWrite(13, !flash);
    flashTimeout = flashTimeNow + flashSpeed;  //reset the timer
    //printTimeVars();
  }
  for (int i = 0; i < NO_OF_POINTS; i++) {
    int onHueStop = 0;  // red
    int onHueGo = 90;   // green
    int neoNum = 2 * i;
    if (POINT_PAIRS[i] != i && pointPairing) {
      onHueStop = 32;  // Orange
      onHueGo = 128;   // aqua
    }
    if (point[i].swPos == 0) {                                       // closed
      leds[LEDS_MIMIC[neoNum]] = CHSV(onHueGo, onSat, onLev);        // even neopixels are the closed route - 90 is green
      leds[LEDS_MIMIC[neoNum + 1]] = CHSV(onHueStop, onSat, onLev);  //, odd neopixels are for the thrown route - 0 is red.
    } else {
      leds[LEDS_MIMIC[neoNum]] = CHSV(onHueStop, onSat, onLev);    // even neopixels are the closed route - 0 is red.
      leds[LEDS_MIMIC[neoNum + 1]] = CHSV(onHueGo, onSat, onLev);  //, odd neopixels are for the thrown route -90 is green
    }
    //if moving

    if (!(point[i].curPos == point[i].thrownPos || point[i].curPos == point[i].closedPos)) {  // point moving
      // printf("  moving point %d. Switch = %d, Cur = %d Clo = %d thrwn = %d millis = ", i, point[i].swPos, point[i].curPos, point[i].closedPos, point[i].thrownPos);
      // Serial.println(millis());
      if (flash) {
        leds[LEDS_MIMIC[neoNum]] = CHSV(160, onSat, 255);
      } else {
        leds[LEDS_MIMIC[neoNum + 1]] = CHSV(160, onSat, 255);
      }
    }


    if (point[i].autoControl) {  // flash off if under PC control
      if (flash) {
        leds[LEDS_MIMIC[neoNum]] = CHSV(160, onSat, 0);
      } else {
        leds[LEDS_MIMIC[neoNum + 1]] = CHSV(160, onSat, 0);
      }
    }
    //if calibrating

    if (cal) {
      int ledatlpm = lastPointMoved * 2;
      //printf("last point moved = %d  ", lastPointMoved);
      if (lastPointMoved < NO_OF_POINTS) {
        leds[LEDS_MIMIC[ledatlpm]] = CHSV(200, onSat, 255);  // purple if calibrating
        //printf("ledsMimic number closed for point %d is  = %d\n", lastPointMoved, LEDS_MIMIC[ledatlpm]);
      } else {
        ledatlpm = ((lastPointMoved - NO_OF_POINTS) * 2) + 1;
        leds[LEDS_MIMIC[ledatlpm]] = CHSV(200, onSat, 255);  // purple if calibrating
        //printf("ledsMimic number thrown for point = %d is %d\n", lastPointMoved-NO_OF_POINTS, LEDS_MIMIC[ledatlpm]);
      }
    }
    if (centreServo) {
      int ledatlpm = lastPointMoved * 2;
      if (lastPointMoved >= NO_OF_POINTS) {
        ledatlpm = ((lastPointMoved - NO_OF_POINTS) * 2);
      }
      onLev = flash * 200;
      leds[LEDS_MIMIC[ledatlpm]] = CHSV(60, onSat, onLev);      // YELLOW
      leds[LEDS_MIMIC[ledatlpm + 1]] = CHSV(60, onSat, onLev);  // YELLOW
    }
  }
  byte sensStarti = NO_OF_POINTS * 2;
  for (uint8_t i = sensStarti; i < NO_OF_LEDS; i++) {
    onHue = 60;
    if (i < NO_OF_BLOCKS + sensStarti) {
      onHue = 230;
    }
    onLev = 0;
    if ((incomingSensors >> (i - sensStarti)) & 1) onLev = 230;
    if (i2cError > 0) {
      onLev = flash * 100;
      onHue = 20;
    }
    leds[LEDS_MIMIC[i]] = CHSV(onHue, onSat, onLev);
  }
  FastLED.show();
  onLev = 100;
}


//--------------------------LED TESTING FOR DEBUGGING-------------------
void testLeds() {

  for (int i = 0; i < NO_OF_POINTS; i++) {
    int neoNum = 2 * i;
    leds[LEDS_MIMIC[neoNum]] = CHSV(160, onSat, 0);
    leds[LEDS_MIMIC[neoNum + 1]] = CHSV(160, onSat, 0);
  }
  FastLED.show();
  delay(1000);
  for (int i = 0; i < NO_OF_POINTS; i++) {
    int neoNum = 2 * i;
    leds[LEDS_MIMIC[neoNum]] = CHSV(64, 255, 250);
    leds[LEDS_MIMIC[neoNum + 1]] = CHSV(128, 255, 250);

    printf("i = %d", i);

    // if (i != 0) {
    //   leds[i - 1] = CHSV(160, 255, 150);
    // }
    FastLED.show();
    delay(1000);
  }
}


//============================ CALIBRATING ============================
void calibrate() {
  bool longPress = 0;
  int pressCount = 0;

  if (digitalRead(ENCODER_PUSH) == 0) {  // it's a press
    pressCount++;
    unsigned long timepressed = millis();
    while (digitalRead(ENCODER_PUSH) == 0) {
      if (millis() - timepressed > LONG_PUSH) {  // long press
        longPress = 1;
        digitalWrite(CAL_LED, cal);  // turns on led if not calibrating, otherwise turns off led
      }
      wdt_reset();  // Reset watchdog to prevent reset
    }               // released
    if (!cal) {     // IF NOT CALIBRATING and encoder pushed

      //OVERRIDING POINT PAIRING BY 3 FAST ENCODER PRESSES WHEN NOT CALIBRATING
      // //toggle point pairing and write to eeprom. Only possible if not calibrating.
      // //checking for a further 3 fast press/release within 3 seconds
      while (millis() < timepressed + LONG_PUSH / 2) {     // 1.5 seconds - can't be a long press
        if (digitalRead(ENCODER_PUSH) == 0) pressCount++;  //count a press
        while (digitalRead(ENCODER_PUSH) == 0) {
          wdt_reset();  // Reset watchdog to prevent reset
        }               //wait for release
        delay(10);
        //wdt_reset();  // Reset watchdog to prevent reset                                     //debounce
      }  // loop back for next fast press if there is time
      if (pressCount >= 5)
        while (true) {}       //reset using watchdog reboots. no exit necessary
      if (pressCount >= 4){ // change movespeed
      changeMoveSpeed = true;

      } else if (pressCount >= 3 && !changeMoveSpeed) {  //centre servos
        centreServo = !centreServo;
        printf("pressCount = %d, changing servo centering to  %d\n", pressCount, centreServo);
      } else if (pressCount >= 2 && !changeMoveSpeed && !centreServo) {
        longPress = 0;  // just to be sure!
        pointPairing = !pointPairing;
        printf("pressCount = %d, changing pointPairing to %d\n", pressCount, pointPairing);
        savePointValues();  // should only write pointPairing as no point calibration positions have changed.
      }
      //printf("pressCount = %d, pointPairing = %d\n", pressCount, pointPairing);
      if (longPress) {  // start calibrating
        cal = 1;
        encoderPos = 0;    // reset
        encoder.write(0);  // reset zero encoder value
        digitalWrite(CAL_LED, 0);
      }
    } else {    //IF ALREADY CALIBRATING when encoder pushed
      cal = 0;  // stop calibrating whether long or short press
      digitalWrite(CAL_LED, 1);
      if (longPress) {
        savePointValues();
      } else {
        loadPointValues();
      }
    }
  }
  if (cal) {  // at any time
    static uint8_t pointNum;
    int32_t move = (encoder.read() - encoderPos);            //current value - old value
    if (move != 0 || oldLastPointMoved != lastPointMoved) {  //there has been an encoder move since last pass (could be -ve) or new point switched
      move *= 4;
      encoder.write(0);  // reset encoder
                         // this is the point number extracted from lastPointMoved

      if (lastPointMoved >= NO_OF_POINTS) {        // this is a point set at thrown
        pointNum = lastPointMoved - NO_OF_POINTS;  // taking 16 of this number
        point[pointNum].thrownPos += move;
        if (point[pointNum].thrownPos < BOTTOM_PULSE_LEN) point[pointNum].thrownPos = BOTTOM_PULSE_LEN;
        if (point[pointNum].thrownPos > TOP_PULSE_LEN) point[pointNum].thrownPos = TOP_PULSE_LEN;
        servo.writeMicroseconds(pointNum, point[pointNum].thrownPos);
        point[pointNum].curPos = point[pointNum].thrownPos;
        printf("Cal point %d : thrown pos = %d \n", pointNum, point[pointNum].thrownPos);
      } else {                      //otherwise this is a point set at closed
        pointNum = lastPointMoved;  // not taking 16 of this number
        point[pointNum].closedPos += move;
        if (point[pointNum].closedPos < BOTTOM_PULSE_LEN) point[pointNum].closedPos = BOTTOM_PULSE_LEN;
        if (point[pointNum].closedPos > TOP_PULSE_LEN) point[pointNum].closedPos = TOP_PULSE_LEN;
        servo.writeMicroseconds(pointNum, point[pointNum].closedPos);
        point[pointNum].curPos = point[pointNum].closedPos;
        printf("Cal point %d : closed pos = %d \n", pointNum, point[pointNum].closedPos);
      }
      oldLastPointMoved = lastPointMoved;
    }

    if (point[pointNum].curPos == BOTTOM_PULSE_LEN || point[pointNum].curPos == TOP_PULSE_LEN) {
      digitalWrite(CAL_LED, flash);
    } else {
      digitalWrite(CAL_LED, 1);
    }
  }
}

//================================= setServoCentre ========================================
void setServoCentre() {
  //if (oldswStatus != swStatus) {
    int lpm = lastPointMoved;
    if (lastPointMoved >= NO_OF_POINTS) {
      lpm = ((lastPointMoved - NO_OF_POINTS));
    }
    for (int i = 0; i < NO_OF_POINTS; i++) {
      if (i == lpm) {
        servo.writeMicroseconds(lpm, 1500);  //move servo
        point[lpm].curPos = 1500;
      } else {
        point[i].movePoint(i);
      }
    }
    printf("Centering point %d\n", lpm);
 // }
}

//================================= i2cReadWrite ========================================

void i2cReadWrite() {

  Wire.beginTransmission(slaveAddress);
  i2cError = Wire.endTransmission();  // Check for errors
  if (i2cError == 0) {
    Wire.requestFrom(slaveAddress, 4);  // Request 16 bits for sensors from Arduino and 16 JMRI point settings
    unsigned long startTime = millis();
    while (Wire.available() < 4) {        // Wait for data, with timeout
      if (millis() - startTime > 1000) {  // 1-second timeout
        Serial.println("Timeout waiting for slave response.");
        return;  // Exit the function to avoid hanging
      }
    }

    // Read incoming data if available
    incomingSensors = Wire.read() << 8 | Wire.read();
    incoming = Wire.read() << 8 | Wire.read();

    // Now send data to the slave
    delay(100);  // Optional, depends on system timing requirements
    Wire.beginTransmission(slaveAddress);
    Wire.write(highByte(swStatus));     // Send high byte
    Wire.write(lowByte(swStatus));      // Send low byte
    i2cError = Wire.endTransmission();  // Check for errors

    if (i2cError != 0) {
      Serial.print("Error writing to slave. I2C Error code: ");
      Serial.println(i2cError);
    }
  } else {
    Serial.print("Error communicating with slave. I2C Error code: ");
    Serial.println(i2cError);
  }
}


//==========================================================================
//===================== SETUP ==============================================
//==========================================================================
void setup() {
  Serial.begin(19200);
  servo.begin();
  servo.setPWMFreq(50);
  yield();

  printf("\npointControlMiniplank Steve Lomax 12/10/24 Free for personal use.\n");
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NO_OF_LEDS);

  for (int i = 0; i < NO_OF_ROWS; i++) {
    pinMode(ROW_PINS[i], OUTPUT);  // 6
  }
  for (int i = 0; i < NO_OF_COLS; i++) {
    pinMode(COL_PINS[i], INPUT);  // A0 A1
  }
  pinMode(ENCODER_PUSH, INPUT_PULLUP);  //5
  pinMode(MOV_LED, OUTPUT);             //13
  pinMode(CAL_LED, OUTPUT);             //11

  loadPointValues();
  if (pointPairing > 254) pointPairing = 1;
  if (pointPairing < 255) pointPairing = 0;

  bool validation = true;
  if (moveSpeed < 1) {
    moveSpeed = 100;
    validation = 0;
  }


  for (int i = 0; i < NO_OF_POINTS; i++) {
    if (point[i].thrownPos < BOTTOM_PULSE_LEN || point[i].thrownPos > TOP_PULSE_LEN) {
      point[i].thrownPos = 2000;
      validation = false;
    }
    if (point[i].closedPos < BOTTOM_PULSE_LEN || point[i].closedPos > TOP_PULSE_LEN) {
      point[i].closedPos = 2000;
      validation = false;
    }
  }
  if (!validation) {
    savePointValues();
  }
  Wire.begin();
  Serial.println("Scanning for I2C Devices...");
  byte error, address;
  int nDevices;
  for (address = 1; address < 127; address++) {
    //communicate with each address
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      if (address == slaveAddress) Serial.print(" sensor Arduino");
      else if (address == 0x40) Serial.print(" PCA9685 servo driver ");
      else Serial.print(" unknown device");

      Serial.println(" !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("device error at address 0x");
      if (address < 16) {
        Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan done\n");
  }

  readSwitches();
  for (int i = 0; i < NO_OF_POINTS; i++) {
    if (point[i].swPos == 1) {
      point[i].curPos = point[i].thrownPos;
    } else {
      point[i].curPos = point[i].closedPos;
    }
  }


  digitalWrite(CAL_LED, 1);
  // delay(500);
  // digitalWrite(CAL_LED, 0);
  // delay(500);

  Serial.println("\nPoint\tCurPos\tClosd\t  Swit\tThrwn ");
  for (int i = 0; i < NO_OF_POINTS; i++) {

    printf(" %d\t%d\t%d\t%d\t%d\n", i, point[i].curPos, point[i].closedPos, point[i].swPos, point[i].thrownPos);
  }
  printf("moveSpeed = %d\n", moveSpeed);
  printf("pointPairing = %d\n", pointPairing);

  //moveSpeed = 100;
  if (digitalRead(ENCODER_PUSH) == 0) {
    changeMoveSpeed = true;
    encoder.write(0);
    printf("change move speed...\n");
  }
  Serial.println("\ndone setup");
  Serial.end();
  //delay(1000);
  bus.begin(19200);
  // digitalWrite(CAL_LED, 1);
  // delay(400);


  while (digitalRead(ENCODER_PUSH) == 0) {}

  // Enable watchdog with a 2-second timeout
  wdt_enable(WDTO_2S);
}



//==========================================================================
//===================== LOOP ===============================================
//==========================================================================
void loop() {
  wdt_reset();  // Reset watchdog to prevent reset
  //testLeds();
  //timeNow = millis();
  if (oldswStatus != swStatus) {  // if there has been a switch change
    printOutData();
  }
  readSwitches();
  //printTimeVars();
  if (centreServo) {  //if in centre servo mode(enc pressed 4 times)
    setServoCentre();
  } else {
    if (!cal) {    //if not in calibration or in centreServo mode
      moving = 0;  // initialise here. moving will be changed in the point struct if any point moves
      for (int i = 0; i < NO_OF_POINTS; i++) {
        point[i].movePoint(i);
      }
      if (!moving) {  // keeps moving smooth so that cmri poll doesnt slow it down
        //getSetData();
        i2cReadWrite();
      }
    }
  }

  if (!changeMoveSpeed) {  // not changing the move speed
    calibrate();
  } else {
    pointMoveSpeed();
  }
  setLeds();
}

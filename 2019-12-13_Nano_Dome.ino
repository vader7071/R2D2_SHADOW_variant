/*
* Arduino Nano
* ATmega 328P
* 
* 2019-12-13
* 
* R2 Dome LED and Servo Controller
* Uses:
*   (4) Adafruit 8x8 matrix (PID 870)
*   (3) Adafruit 16x8 matrix (PID 3152)
*   (1) Adafruit I2C PWM 16CH breakout Board (PID 815)
*   (2) Tri-color LED assemblies
* 
* Body Arduino Mega communicates to this unit via serial comm.
* 
* Serial Comm Port is pins 12 & 13 on this processor
* Holoemitter servos not programmed yet.  
* 
*/

//============  Includes/Defines  ============
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

#define MATRICES_8x8 4
#define MATRICES_16x8 3
#define SERVOMIN  150                                                   // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600                                                   // this is the 'maximum' pulse length count (out of 4096)
//#define DEBUG_ON

Adafruit_8x8matrix matrix8[MATRICES_8x8];
Adafruit_8x16minimatrix matrix16[MATRICES_16x8];
Adafruit_PWMServoDriver holoServo = Adafruit_PWMServoDriver();          // I2C address of 0x40
SoftwareSerial comPort(12,13);                                          // Create second serial port.  Pin 2 = RX, 3 = TX

//============  Constants  ============
/*
 *Activate Pins Used 
 */
// --------- Digital Pins ---------
//int [pin] = 0;              // DNU - RX PIN
//int [pin] = 1;              // DNU - TX PIN
//int [pin] = 2;              // Dig I/O
int front_psi_a = 3;          // Dig I/O         PWM
//int [pin] = 4;              // Dig I/O            
int front_psi_b = 5;          // Dig I/O         PWM
int front_psi_c = 6;          // Dig I/O         PWM
//int [pin] = 7;              // Dig I/O
//int [pin] = 8;              // Dig I/O
int rear_psi_a = 9;           // Dig I/O         PWM
int rear_psi_b = 10;          // Dig I/O  SS     PWM
int rear_psi_c = 11;          // Dig I/O  MISO   PWM
//int [pin] = 12;             // Dig I/O  MOSI
//int [pin] = 13;             // Dig I/O  SCK

// --------- Analog Pins ---------
//int [Apin] = A0;            // Analog In 0
//int [Apin] = A1;            // Analog In 1
//int [Apin] = A2;            // Analog In 2
//int [Apin] = A3;            // Analog In 3
int I2C_SDA = A4;             // Analog In 4
int I2C_SCL = A5;             // Analog In 5
//int [Apin] = A6;            // Analog In 6
//int [Apin] = A7;            // Analog In 7

/*
 * ICSP PIN LAYOUT
 * 
 * Rst            Rst
 * Gnd   2 4 6*   RX0
 * Vin   1 3 5    TX1
 *    
 * 1 = MOSI (aka pin 12)
 * 2 = 5V
 * 3 = SCK  (aka pin 13)
 * 4 = MISO (aka pin 11)
 * 5 = RST  (Not In Use)
 * 6 = GND
 *  
 */

// -------- Data Displays --------
const int data_time_delay[2] = {20,50};                                 // [0] = random timer, [1] = scroll message
char* msg[] = {"","R2DPool Rules!!!       ","Suck it Wolverine!      ","Where's Francis?      ","Don't be a Dick!      "};
const int numOfMsg = 5;
const int SERVOMAX = 4095;
const int SERVOMIN = 0;
/*
 * If the quantity of messages changes, the constant variable "maxMsg" in the ADK processor sketch must be updated as well.
 * TODO:  setup the Nano to transmit via serial the number of msg to the ADK to keep it up to date.
 *        idea:  set ADK to monitor serial data unless TX.  Set Nano to TX "numOfMsg" to ADK after each msg change.
 */

//============  Variables  ============

// -------- Arrays --------
// -------- PSI --------
unsigned long prev_milli[3] = {millis()};                               // [0] & [1] psi, [2] = scroll message
unsigned long hp_prev_milli[6] = {millis()};                            // used to time for all 6 HP servos

int psi_value[2][3];                                                    // array setup [x][y] => x = front vs rear; y = color a, b, or c.  
bool psi_rising[2][3] = {true};                                         // array [x][y] => "X" 0 = front, 1 = rear
int psi_LED[2][3];                                                      // Used for initializing
int psi_time_delay[2][2] = { {8,2000},{8,2000} };                       // time delays for color changes
int psi_mark[2] = {0};                                                  // holds value of which LED is being affected [0{r}, 1{b}, 2{g}]
int hp_time_delay[6] = {0};

// -------- Serial Data --------
uint8_t msgData = 0;
int runCount = 0;

//============  Setup Function  ============
void setup() {
  // -------- Begin Comms --------
  #ifdef DEBUG_ON
    Serial.begin(9600);
  #endif
  comPort.begin(9600);
  #ifdef DEBUG_ON
    Serial.println(F("SoftSerial Comm Port Started"));
  #endif
    
  // -------- Array filling --------
  #ifdef DEBUG_ON
    Serial.println(F("Begin filling PSI LED Array"));
  #endif
  psi_LED[0][0] = front_psi_a;
  psi_LED[0][1] = front_psi_b;
  psi_LED[0][2] = front_psi_c;
  psi_LED[1][0] = rear_psi_a;
  psi_LED[1][1] = rear_psi_b;
  psi_LED[1][2] = rear_psi_c;
  #ifdef DEBUG_ON
    Serial.println(F("Array data:"));
    Serial.print(F("Array Block 0, slot 0: ")):
    Serial.println(psi_LED[0][0]);
    Serial.print(F("Array Block 0, slot 1: ")):
    Serial.println(psi_LED[0][1]);
    Serial.print(F("Array Block 0, slot 2: ")):
    Serial.println(psi_LED[0][2]);
    Serial.print(F("Array Block 1, slot 0: ")):
    Serial.println(psi_LED[1][0]);
    Serial.print(F("Array Block 1, slot 1: ")):
    Serial.println(psi_LED[1][1]);
    Serial.print(F("Array Block 1, slot 2: ")):
    Serial.println(psi_LED[1][2]);
  #endif

  // -------- Setup Random Generator --------
  randomSeed(analogRead(0));
  
  // -------- Setup Outputs --------
  for (int x = 0; x < 2; x++){
    for (int y = 0; y < 3; y++){
      pinMode(psi_LED[x][y], OUTPUT);
    }
  }
  #ifdef DEBUG_ON
    Serial.println(F("LED Outputs initialized as OUTPUT"));
  #endif

  // -------- Setup 8x8 --------
  for(uint8_t m=0; m<MATRICES_8x8; m++) {                               // Set the address and mode for the 8x8 matrices
    matrix8[m].begin(0x70 + m);
    matrix8[m].clear();
    matrix8[m].setTextSize(1);
    matrix8[m].setTextWrap(false);                                      // we dont want text to wrap so it scrolls nicely
    matrix8[m].setRotation(0);                                          // 0 = Conn @ Top; 1 = Conn @ Left; 2 = Conn @ Bottom; 3 = Conn @ Right
  }
  #ifdef DEBUG_ON
    Serial.println(F("8x8 Matrices Initialized"));
  #endif

  // -------- Setup 16x8 --------
  for(uint8_t d = 0; d < MATRICES_16x8; d++) {                          // Set the address and mode for the 8x8 matrices
    matrix16[d].begin(0x74 + d);
    matrix16[d].clear();
    matrix16[d].setTextSize(1);
    matrix16[d].setTextWrap(false);                                     // we dont want text to wrap so it scrolls nicely
    matrix16[d].setRotation(1);                                         // 0 = Conn @ Top; 1 = Conn @ Left; 2 = Conn @ Bottom; 3 = Conn @ Right
  }
  #ifdef DEBUG_ON
    Serial.println(F("16x8 Matricies Initialized"));
  #endif

  // -------- Setup 16ch Servo Driver --------
  holoServo.begin();
  #ifdef DEBUG_ON
    Serial.println(F("Servos initialized"));
  #endif
  holoServo.setPWMFreq(60);                                             // This is the maximum PWM frequency
  #ifdef DEBUG_ON
    Serial.println(F("PWM Freq set to: 60"));
  #endif

  // -------- Setup HP time delay --------
  for (int h = 0; h < 6; h++) {
    hp_time_delay[h] = random(10000,60000);
  }
}

//============  Loop Function  ============
void loop() {
  if (comPort.available() > 0) {
    msgData = comPort.read();
    #ifdef DEBUG_ON
      Serial.println(msgData);
    #endif
  }
  dataDisplay();                                                        // Find way to make scroll() not so processor hungry
  psi();                                                                 // Will I need to call PSI out of scroll()?  Scroll is a pretty demanding subloop
  //holoEmitters();                                                     // Same. When servos installed, will I need to   
}

//============  Data Display Function  ============
void dataDisplay() {
  if (msgData > 0 && msgData < numOfMsg){
    scroll(msg[msgData]);
  } else {
    randomData();
  }
}

//------ Random Data ------
void randomData() {
  if (millis() - prev_milli[2] >= data_time_delay[0]){                  // Time based delay            
    // --- 8x8 Matrix ---
    for (int e = 0; e < MATRICES_8x8; e++) {                            // Scroll through each module
      matrix8[e].clear();                                               // Clear all previous data
      for (int row8 = 0; row8 < 8; row8++) {                            // scroll through each row
        for (int col8 = 0; col8 < 8; col8++) {                          // scroll through each bit per row
          matrix8[e].drawPixel(row8, col8, random(0,2));                // randomly pick 1 or 0 for each LED
        }
      }
      matrix8[e].writeDisplay();                                        // Write all data to the module
    }
    
    // --- 16x8 Matrix ---
    for (int f = 0; f < MATRICES_16x8; f++) {
      matrix16[f].clear();
      for (int row16 = 0; row16 < 16; row16++) {
        for (int col16 = 0; col16 < 8; col16++) {
          matrix16[f].drawPixel(row16, col16, random(0,2));
        }
      }
      matrix16[f].writeDisplay();
    }
    prev_milli[2] = millis();
  }
}

//------ Scrolling Message ------
/*
 * see 2019-10-17_R2_Dome_Data_Ports_Scroll_Test sketch for code.  Scroll works, but front 2 data displays are blank.  No display during message scroll.
 * Still in progress
 */
void scroll(char* text) {
  for (int f = 1; f < MATRICES_16x8; f++){
    matrix16[f].clear();
    matrix16[f].writeDisplay();
  }
  int scrollPositions = (strlen(text) * 8);
  for (int x = (8 * (MATRICES_8x8 + 1)); x >=- scrollPositions; x--) {  // To add the 16x8, make sure line reads: for (int x=(8 * (MATRICES_8x8 + 2)); x>=-scrollPositions; x--)
    if (millis() - prev_millis[2] >= data_time_delay[1]){
      for(uint8_t m = 0; m <=  MATRICES_8x8; m++) {                     // To add the 16x8, make sure line reads: for(uint8_t m = 0; m = MATRICES_8x8; m++)
        if(m < MATRICES_8x8){
          matrix8[m].clear();
          matrix8[m].setCursor((x - (m * 8)), 0);
          matrix8[m].print(text);
          matrix8[m].writeDisplay();
        } else {
          matrix16[0].clear();
          matrix16[0].setCursor((x - (m * 8)), 0);
          matrix16[0].print(text);
          matrix16[0].writeDisplay();
        }
      }
      prev_milli[2] = millis();
    }
  }
  runCount++;
  if (runCount > 2){
    runCount = 0;
    msgData = 0;
    int dataDump = numOfMsg - 1;
    comPort.print(dataDump);
  }
}

//============  Processor State Function  ============
/*
 * 2 simple round lights.  Front is under Radar Eye
 * PSI = Processor State Indicator
 */
void psi() {
  // psi_mark[*] = which LED is being driven (R, G, or B).  X = front/rear
  for (int x = 0; x < 2; x++){
    if (millis() - prev_milli[x] >= psi_time_delay[x][0]) {
      if (psi_rising[x][psi_mark[x]] == true && psi_value[x][psi_mark[x]] < 255) {
        psi_value[x][psi_mark[x]]++;
      }
      if (psi_rising[x][psi_mark[x]] == false && psi_value[x][psi_mark[x]] > 0) {
        psi_value[x][psi_mark[x]]--;
        if (psi_value[x][psi_mark[x]] <= 85) {
          if (psi_mark[x] < 2 ) {
            psi_value[x][psi_mark[x+1]]++;
          } else {
            psi_value[x][0]++;
          }
        }
      }
      if (psi_value[x][psi_mark[x]] < 255 && psi_value[x][psi_mark[x]] > 0) {
        prev_milli[x] = millis();
      }
    }  
    if (psi_value[x][psi_mark[x]] >= 255 && (millis() - prev_milli[x] >= psi_time_delay[x][1])) {
      psi_rising[x][psi_mark[x]] = false;
    }
    if (psi_value[x][psi_mark[x]] <= 0 && (millis() - prev_milli[x] >= psi_time_delay[x][1])) {
      psi_rising[x][psi_mark[x]] = true;
      if (psi_mark[x] < 2 ) {
        psi_mark[x]++;
      } else {
        psi_mark[x] = 0;
      }
    } 
  }
  
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 3; y++) {
      analogWrite(psi_LED[x][y], psi_value[x][y]);
    }
  }
}

//============  Servo Position Function  ============
void holoEmitters() {
  /*
  for (int z = 0; z < 6; z++){
    if (millis() - hp_prev_milli[z] >= hp_time_delay[z]) {
      int pulselength = map(random(0,90), 0, 180, SERVOMIN, SERVOMAX);
      holoServo.setPWM(z, 0, pulselength);
      hp_time_delay[z] = random(750,5000);
      hp_prev_milli[z] = millis();
    }
  }

  Converting from Degrees to Pulse Length
The Arduino "map()" function is an easy way to convert between degrees of rotation and your 
calibrated SERVOMIN and SERVOMAX pulse lengths.  Assuming a typical servo with 180 degrees 
of rotation; once you have calibrated SERVOMIN to the 0-degree position and SERVOMAX to the 
180 degree position, you can convert any angle between 0 and 180 degrees to the corresponding 
pulse length with the following line of code:
pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  */
}

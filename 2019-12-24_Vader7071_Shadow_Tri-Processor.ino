// =====================================================================================
//                 SHADOW :  Small Handheld Arduino Droid Operating Wand
// =====================================================================================
//                 Vader7071's Modification 2019-12-24
//
//    Chris Wren vader7071@darksidev2.com
//
//    System is hard coded to use only 1 PS3 Move Navigation controller.
//    *  Primary Processor is an Arduino Mega 2560 ADK.
//    *  Dome Processor is an Arduino Nano controlled from the Primary processor via serial data.
//    *  Dome Data Display is Adafruit 0.8"x0.8" mini LED matrix (PID 870 & 3152) - I2C data bus from the Dome Processor
//    *  Dome PWM is Adafruit PWM I2C breakout (PID 815) - I2C data bus from the Dome Processor
//    *  Sound Processor is an Arduino Leonardo controlled from the Primary processor via serial data.
//    *  Sound controller is Adafruit Music Maker Shield (PID 1788) - SPI data bus from a Leonardo Processor
//    *  Audio Amp is Adafruit 20W Amp (PID 1752) - I2C data bus (Volume Control) from the Sound Processor
//    *  Sabertooth 2x32A - Serial Data bus (tx = p16 on Mega) from the Primary Processor
//    *  Syren 25A - Serial Data bus (tx = p16 on Mega) from the Primary Processor
//
//    System is designed to have Mega in body and Nano (or other Arduino) inside dome.
//    Data is transmitted via serial from Mega to Nano to control dome functions.
//
//    Adjusted A LOT of variables to use array format.
// =====================================================================================
//         Last Revised Date: 4/15/19
//         Written By: KnightShade
//         Inspired by the PADAWAN by danf
//         Bug Fixes from BlackSnake and vint43
//         Contributions for PWM Motor Controllers by JoyMonkey/Paul Murphy w/ credit to Brad/BHD
//         LED & Utility Arm control contribution by Dave C.
//            Notes by Vader7071:
//            Whe Dave C.'s utility arm code got added, he used a couple of variables 'const int UTILITY_ARM_TOP=1' and 'const int UTILITY_ARM_BTM=2'
//            Looking in the body section, there is the portion for "Utility Arms".  I would make the assumption that Dave created these variables
//            because he was using a Switch Case command structure, and by doing it this way, he could easily add new "utility arms" just by adding
//            new cases in the code.  To add more servos and arm controls, expand the arrays and add new "UTILITY_ARM_*** = #" to match the case.
//         
// =====================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// =====================================================================================
//   Note: You will need a Arduino Mega 1280/2560 to run this sketch,
//   as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//      - Single PS3 Move Navigation
//      - Android Phone (Limited Controls)
//
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/ or
//
//   Sabertooth (Foot Drive):
//         Set Sabertooth 2x32 or 2x25 Dip Switches: 1 and 2 Down, All Others Up
//
//   SyRen 10 Dome Drive:
//         For SyRen packetized Serial Set Switches: 1, 2 and 4 Down, All Others Up
//         NOTE:  Support for SyRen Simple Serial has been removed, due to problems.
//         Please contact DimensionEngineering to get an RMA to flash your firmware
//         Some place a 10K ohm resistor between S1 & GND on the SyRen 10 itself
//
// =====================================================================================
//
// =====================================================================================
//                          User Settings, Variables, Includes, & Declarations
// =====================================================================================
/*  BUTTON MAP:
 *	Joystick
 *		Left/Right:			Unmodified: Drive Left or Right, Modified by L3: turns Dome
 *		Up/Down:			  Unmodified: Drive Forward or Backward, Modified by L3: {nothing}
 *	
 *	Buttons/Direction Pad
 *		Cross (X):			UnMod: Dome LT, L1 Top Utility Arm
 *		Circle (O):			UnMod: Dome RT, L1 Btm Utility Arm
 *    
 *		Up:					    Sound Control (modified by L1 & L2)
 *		Down:				    Sound Control (modified by L1 & L2)
 *		Left:				    Sound Control (modified by L1 & L2)
 *		Right:				  Sound Control (modified by L1 & L2)
 *		PS:					    UnMod: Change Dome Data, Mod L1: *****, Mod L2: *****, Mod L3: Disconnect
 *
 *	Modifier Buttons
 *		L1:					    Modifier
 *		L2:					    Modifier
 *		L3:					    Modifier
 *    
 *    Completed:
 *      Dome spin via L1+Cross/Circle
 *      Dome spin via L3+Joystick
 *      Utility Arms
 *      Sound Control
 *      Dome Data Display Port Message Control
 *      Coin Slot Flashing
 *      Control Toggle
 */

// -------------------------------------------------------------------------------------
//                          Includes/Defines
// -------------------------------------------------------------------------------------
// ---- Includes ----
#include <PS3BT.h>                                            // Used for PS3 Controller
#include <usbhub.h>                                           // Used for PS3 Controller
#include <LedControl.h>                                       // For LEDs in Coin Slots
#include <LiquidCrystal_I2C.h>                                // For LCD Display

// Satisfy IDE, which only needs to see the include statment in the ino.
 #ifdef dobogusinclude
 #include <spi4teensy3.h>
 #endif

// ---- Speed Controllers Includes
#include <Sabertooth.h>                                       // For Drive Elec Speed Controller (ESC)
#include <Servo.h>                                            // For utility arm servos

// ---- Defines ----
#define numberOfCoinSlotLEDs 6                                // Used for Body Coin Slots LED Control
#define SYREN_ADDR         129                                // Serial Address for Dome Syren
#define SABERTOOTH_ADDR    128                                // Serial Address for Foot Sabertooth
#define FOOT_CONTROLLER 0                                     // Drive Control - 0 for Sabertooth Serial, 1 for individual R/C output

//#define TEST_CONROLLER                                      // Support coming soon
#define DEBUG_ON                                            // uncomment this for console DEBUG output
#define SHADOW_VERBOSE                                      // uncomment this for console VERBOSE output
//#define BLUETOOTH_SERIAL                                    // uncomment this for console output via bluetooth.  
                                                              // NOTE:  BLUETOOTH_SERIAL is suspected of adding CPU load in high traffic areas

// -------------------------------------------------------------------------------------
//                          Control System Variables/Settings
// -------------------------------------------------------------------------------------
String bluetoothDongleMac = "00:15:83:F3:5E:17";              // This is the MAC address of the Bluetooth Dongle attached to the Arduino
String PS3MoveNavigatonPrimaryMAC = "E0:AE:5E:A5:0E:40";      // This is the MAC address of the *PRIMARY* PS3 Move Navigation Controller

// -------------------------------------------------------------------------------------
//                          Constants
// -------------------------------------------------------------------------------------
// --------- Arrays ---------
const int COIN_SLOT_LED_PINS[] = { 22, 23, 24, 25, 26, 27 };  // Coin Slot LEDS - LED pins to use for Coin Slot
const int serialBaudRate[2] = {9600,9600};                    // SyRen & Sabertooth - [0] = ESC, [1] = comm to additional arduinos.  Set the baud rate for the Syren motor controller for packetized options are: 2400, 9600, 19200 and 38400
const int UTILITY_ARM_PIN[2] = {8, 9};                        // Utiltity Arm - UAP[0] = Top, UAP[1] = Bottom
const int utilArmMinMax[2] = {0, 90};                         // Utiltity Arm - [0] = closed position, [1] = open position
const int joyStickData[3] = {15, 10, 10};                     // Drive Control - For centering problems, use the lowest number with no drift. [0] = Foot, [1] = Dome, [2] = deadband (Used to set the Sabertooth DeadZone for foot motors)
 
// --------- Variables ---------
const int UTILITY_ARM_TOP = 1;                                // Utiltity Arm
const int UTILITY_ARM_BTM = 2;                                // Utiltity Arm
const int invertTurnDirection = -1;                           // Dome Control - This may need to be set to 1 for some configurations
const int syrenTimeout = 300;
const int sbrthTimeout = 300;
// const int domeAutoSpeed = 127;                               // Dome Control - Speed used when dome automation is active (1- 127)
// const int time360DomeTurnLeft = 1000;                        // Dome Control - milliseconds for dome to complete 360 turn at domeAutoSpeed
// const int time360DomeTurnRight = 300;                        // Dome Control - milliseconds for dome to complete 360 turn at domeAutoSpeed
                                                                // CDome Control - ut in half to reduce spin.  Offset for different rotation startups due to gearing.
const int maxMsg = 4;                                         // Number of messages the Dome has stored to display
  
// -------------------------------------------------------------------------------------
//                          Variables
// -------------------------------------------------------------------------------------
//---------- Arrays ----------
long prevMillis[3] = {millis()};                              // Drive Control - prevMillis[0] = foot, [1] = dome, [2] = errorState flashing
bool isMotorStopped[2] = {true};                              // Drive Control - isMotorStopped[0] = foot, [1] = dome
long nextCoinSlotLedFlash[numberOfCoinSlotLEDs];              // Coin Slot LEDS - Array indicating which LED to flash next.
int coinSlotLedState[numberOfCoinSlotLEDs];                   // Coin Slot LEDS - Array indicating the state of the LED's.
int utilArmPos[2] = {0, 0};                                   // Utiltity Arm - [0] = top. [1] = bottom
bool isUtilArmOpen[2] = {false, false};                       // Utiltity Arm - [0] = top, [1] = bottom , open = true, closed = false
int speedData[6] = {25, 65, 75, 100, 3, 0};                   // Drive Control - [0] = drivespeed 1, [1] = drivespeed 2, [2] = turn speed, [3] = dome speed, [4] = ramping, [5] = foot drive speed
/* Notes:  set [0], [1], and [2] to whatever speeds work for you. 0-stop, 127-full speed.  Recommend beginner: 50 to 75, experienced: 100 to 127.
 * [2] the higher this number the faster it will spin in place, lower - easier to control.  Recommend beginner: 40 to 50, experienced: 50 & up.
 * [3] If using a speed controller for the dome, sets the top speed.  Use a number up to 127 for serial
 * [4] speedData[4] - the lower this number the longer R2 will take to speedup or slow down, change this by increments of 1
 */

// --------- Variables ---------
long currentMillis = millis();
int serialLatency = 25;                                       // This is a delay factor in ms to prevent queueing of the Serial data.  25ms seems appropriate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation
uint8_t msgData = 0;                                          // Dome Control - Sends data to Dome Processor to determine what message to display
byte sndData = 0;                                             // Sound Control - Sends data to Sound Processor to determine what audio file to play
uint32_t msgLagTime = 0;                                      // Used for PS3 Fault Detection
uint32_t lastMsgTime = 0;                                     // Used for PS3 Fault Detection
uint32_t currentTime = 0;                                     // Used for PS3 Fault Detection
uint32_t lastLoopTime = 0;                                    // Used for PS3 Fault Detection
int badPS3Data = 0;                                           // Used for PS3 Fault Detection
bool isPS3NavigatonInitialized = false;                       // 
bool isStickEnabled = true;                                   // Control bit
bool isAutomateDomeOn = false;                                // Control bit
unsigned long automateMillis = 0;                             // Control bit
bool domeAutomation = false;                                  // Dome Automation - Control bit
int domeTurnDirection = 1;                                    // Dome Automation - 1 = positive turn, -1 negative turn
float domeTargetPosition = 0;                                 // Dome Automation - (0 - 359) - degrees in a circle, 0 = home
unsigned long domeStopTurnTime = 0;                           // Dome Automation - millis() when next turn should stop
unsigned long domeStartTurnTime = 0;                          // Dome Automation - millis() when next turn should start
int domeStatus = 0;                                           // Dome Automation - 0 = stopped, 1 = prepare to turn, 2 = turning
int action = 0;                                               // Control Bit
unsigned long DriveMillis = 0;                                // Drive system time storage
bool wandConnected = false;                                   // wand connected bit
bool errorState = false;                                      // error state bit
bool flashState = false;                                      // LCD flash state bit
String output = "";
String lcdMsg = "  13 Jan 2020";                              // data to be printed on LCD screen
String lcdPrevMsg[] = {"","R2 DPool Activating","SHADOW Ctrl System"," Vader7071 Variant"};

 #ifdef BLUETOOTH_SERIAL
   SPP SerialBT(&Btd,"R2DPool","1977");                       // Create a BT Serial device(defaults: "Arduino" and the pin to "0000" if not set)
   bool firstMessage = true;
 #endif

// -------------------------------------------------------------------------------------
//                          Module Setup/Activations
// -------------------------------------------------------------------------------------
USB Usb;                                                      // Initialize USB for PS3 Controller
BTD Btd(&Usb);                                                // You have to create the Bluetooth Dongle instance like so
//PS3BT *PS3Nav=new PS3BT(&Btd );  // This hard codes the instance of the PS3 Controller for a specific dongle. Use the MAC address for the dongle, not the controller.
PS3BT *PS3Nav=new PS3BT(&Btd,0x00,0x15,0x83,0xF3,0x5E,0x17);  // This hard codes the instance of the PS3 Controller for a specific dongle. Use the MAC address for the dongle, not the controller.
Sabertooth *ST=new Sabertooth(SABERTOOTH_ADDR, Serial2);      // Sabertooth 2x32 Serial Control
Sabertooth *SyR=new Sabertooth(SYREN_ADDR, Serial2);          // SyRen 10A Serial Control
Servo UtilArmTopServo;                                        // create servo object to control a servo 
Servo UtilArmBottomServo;                                     // create servo object to control a servo
LiquidCrystal_I2C lcd(0x27,20,4);                             // set the LCD address to 0x27 for a 20 chars and 4 line display

// =====================================================================================
//                          END User Settings, Variables, Includes, & Declarations
// *************************************************************************************
//                          Setup Function
// =====================================================================================
void setup(){
//  randomSeed(analogRead(0));
  // ------- Setup for LCD Display
  lcd.init();                                                 // initialize the lcd
  //lcd.autoscroll();
  // Print a message to the LCD.
  lcd.backlight();                                            // Turn on backlight 
  lcdPrint(lcdMsg);
  
  // ------- Setup for Serial Output to Serial Monitor
  Serial.begin(9600);                                         // Opens Comm port to laptop for monitoring
  while (!Serial);                                            // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1){
    Serial.println(F("OSC did not start"));
    lcd.clear();
    lcd.setCursor(0,0);                                       // Set cursor to spot 1 on the 1st row
    lcd.print("ERROR");
    lcd.setCursor(0,1);                                       // Set cursor to spot 1 on the 2nd row
    lcd.print("OSC DID NOT START");
    while (1);                                                // halt - Shuts down entire Mega.  Nothing more will happen.
  }
  Serial.println(F("Bluetooth Library Started"));
  output.reserve(200);                                        // Reserve 200 bytes for the output string

  // ------- Setup for PS3
  PS3Nav->attachOnInit(onInitPS3);                            // onInit() is called upon a new connection - you can call the function whatever you like

  // ------- Serial Ports Setup
  /* The Arduino Mega has three additional serial ports: 
   *   Serial1 on pins 19 (RX) and 18 (TX), 
   *   Serial2 on pins 17 (RX) and 16 (TX) - This is set for data path to Dome
   *   Serial3 on pins 15 (RX) and 14 (TX) - This is set for Sabertooth and SyRen
   */
  Serial3.begin(serialBaudRate[0]);                           // Motor Controllers - Syren (Dome) and Sabertooth (Feet)
  #ifdef DEBUG_ON
    Serial.println(F("ESC Serial Started"));
  #endif
  lcdMsg = "ESC Serial Started";
  lcdPrint(lcdMsg);
  Serial2.begin(serialBaudRate[1]);                           // Data path to Dome
  #ifdef DEBUG_ON
    Serial.println(F("Dome Control Serial Started"));
  #endif
  lcdMsg = "Dome Serial Started";
  lcdPrint(lcdMsg);
  Serial1.begin(serialBaudRate[1]);                           // Data path to Sound Controller
  #ifdef DEBUG_ON
    Serial.println(F("Sound Control Serial Started"));
  #endif
  lcdMsg = "SND Serial Started";
  lcdPrint(lcdMsg);

  // ------- SyRen Serial Data Setup - Dome rotation
  SyR->setBaudRate(serialBaudRate[0]);                        // Sets baud rate in SyRen to 9600
  SyR->setTimeout(syrenTimeout);                              // DMB:  How low can we go for safety reasons?  multiples of 100ms
  #ifdef DEBUG_ON
    Serial.println(F("SyRen Data Setup:"));
    Serial.print(F("    Baud: "));
    Serial.println(serialBaudRate[0]);
    Serial.print(F("    Timeout: "));
    Serial.print(syrenTimeout);
    Serial.println(F(" ms"));
  #endif

  // ------- Sabertooth Serial Data Setup - Foot motors
  ST->setBaudRate(serialBaudRate[0]);                         // Sets baud rate in Sabertooth to 9600
  ST->setTimeout(sbrthTimeout);                               // DMB:  How low can we go for safety reasons?  multiples of 100ms
  ST->setDeadband(joyStickData[2]);
  #ifdef DEBUG_ON
    Serial.println(F("Sabertooth Data Setup:"));
    Serial.print(F("    Baud: "));
    Serial.println(serialBaudRate[0]);
    Serial.print(F("    Timeout: "));
    Serial.print(sbrthTimeout);
    Serial.println(F(" ms"));
    Serial.print(F("    Joystick Deadband: "));
    Serial.println(joyStickData[2]);
  #endif
  stopFeet();

  /* NOTE: *Not all* Sabertooth controllers need the autobaud command.
   *       It doesn't hurt anything, but V2 controllers use an
   *       EEPROM setting (changeable with the function setBaudRate) to set
   *       the baud rate instead of detecting with autobaud.
   *
   *       If you have a 2x12, 2x25 V2, 2x60 or SyRen 50, you can remove
   *       the autobaud line and save yourself two seconds of startup delay.
   */

  // ------- Setup for Utility Arm Servo's
  lcdMsg = "Util Arms Initialize";
  lcdPrint(lcdMsg);    
  UtilArmTopServo.attach(UTILITY_ARM_PIN[0]);  
  #ifdef DEBUG_ON
    Serial.println(F("Top Utility Arm Servo Assigned"));
  #endif
  lcdMsg = "Top Arm Set";
  lcdPrint(lcdMsg);
  UtilArmBottomServo.attach(UTILITY_ARM_PIN[1]);
  #ifdef DEBUG_ON
    Serial.println(F("Bottom Utility Arm Servo Assigned"));
  #endif
  lcdMsg = "Btm Arm Set";
  lcdPrint(lcdMsg);
  closeUtilArm(UTILITY_ARM_TOP);                              // Close top arm
  #ifdef DEBUG_ON
    Serial.println(F("Closing Top Utility Arm"));
  #endif
  lcdMsg = "Close Top Arm";
  lcdPrint(lcdMsg);
  closeUtilArm(UTILITY_ARM_BTM);                              // Close bottom arm
  #ifdef DEBUG_ON
    Serial.println(F("Closing Bottom Utility Arm"));
  #endif
  lcdMsg = "Close Btm Arm";
  lcdPrint(lcdMsg);
    
  // ------- Setup for Coin Slot LEDs    
  for(int i = 0; i<numberOfCoinSlotLEDs; i++) {
    pinMode(COIN_SLOT_LED_PINS[i],OUTPUT);
    coinSlotLedState[i] = LOW;
    digitalWrite(COIN_SLOT_LED_PINS[i], LOW);                 // all LEDs off
    nextCoinSlotLedFlash[i] = millis() +random(100, 1000);
  }
  #ifdef DEBUG_ON
    Serial.println(F("Coin Slot LEDs have been initialized"));
  #endif
  lcdMsg = "Coin Slot Initialize";
  lcdPrint(lcdMsg);

  lcdMsg = "Droid Ready";
  lcdPrint(lcdMsg);
  lcdMsg = "Waiting for Ctrlr";
  lcdPrint(lcdMsg);
}

// =====================================================================================
//                          END Setup Function
// *************************************************************************************
//                          Read USB Function
// =====================================================================================
bool readUSB() {
  // The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  Usb.Task();
  if (PS3Nav->PS3NavigationConnected ){
    Usb.Task();
  }
  if ( criticalFaultDetect()){
    // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    // flushAndroidTerminal();
    #ifdef DEBUG_ON
      Serial.println(F("CRITICAL FAULT DETECTED!"));
    #endif
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("    ***ERROR***");
    lcd.setCursor(0,1);
    lcd.print("   CRITICAL FAULT");
    lcd.setCursor(0,2);
    lcd.print("   SYSTEM HALTED");
    lcd.setCursor(0,3);
    lcd.print("  CHECK CONTROLLER");
    wandConnected = false;
    errorState = true;
    return false;
  }
  return true;
}

// =====================================================================================
//                          END Read USB Function
// *************************************************************************************
//                          Loop Function
// =====================================================================================
void loop(){
  if (wandConnected == false){
    if (PS3Nav->PS3NavigationConnected ){
      wandConnected = true;
      errorState = false;
      lcd.display();
      lcdMsg = "Controller Connected";
      lcdPrint(lcdMsg);
    }
  }

  if (errorState == true){
    if (millis() - prevMillis[3] >= 750){
      if (flashState == false){
        lcd.display();
        flashState = true;
      } else {
        lcd.noDisplay();
        flashState = false;
      }
      prevMillis[3] = millis();
    }
  }

  // Useful to enable with serial console when having controller issues.
  #ifdef TEST_CONROLLER
    testPS3Controller();
  #endif

  // LOOP through functions from highest to lowest priority.
  if ( !readUSB()){
    return;                                                                                              //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
  }
  
  footMotorDrive();
        
  if ( !readUSB()){
    return;                                                                                              //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
  }

  // -- Dome Functions --
  // automateDome();
  domeDrive();
  // holoprojector();
  sendMsgData();
    
  // -- Body Functions --
  utilityArms();
  toggleSettings();
  flashCoinSlotLEDs();

  // -- Sound Functions --
  sndCtrl();

  // -- Data updates from slave processessors --
  // - Update number of messages stored in the Dome -
  if (Serial2.available() > 0) {
  	int dataDump = (Serial2.read());
  	if (dataDump != maxMsg) {
      maxMsg = (Serial2.read());
      String temp="Dome Msg Count: "
      lcdMsg = temp + maxMsg;
      lcdPrint(lcdMsg);
    }
  }
}

// =====================================================================================
//                          END Loop Function
// *************************************************************************************
//                          PS3 Controller
// =====================================================================================
void onInitPS3(){
  String btAddress = getLastConnectedBtMAC();
  PS3Nav->setLedOn(LED1);
  isPS3NavigatonInitialized = true;
  badPS3Data = 0;
  #ifdef DEBUG_ON
    output += "\r\nBT Address of Last connected Device when Primary PS3 Connected: ";
    output += btAddress;
    if (btAddress == PS3MoveNavigatonPrimaryMAC){
      output += "\r\nWe have our primary controller connected.\r\n";
    } else {
      output += "\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n";
    }
  #endif
}

String getLastConnectedBtMAC(){
  String btAddress = "";
  for(int8_t i = 5; i >= 0; i--){
    if (btAddress.length() > 0){
      btAddress +=(":");
    }
    if (Btd.disc_bdaddr[i]<0x10){
      btAddress +="0";
    }
    btAddress += String(Btd.disc_bdaddr[i], HEX);
  }
  btAddress.toUpperCase();
  return btAddress; 
}

// =====================================================================================
//                          END PS3 Controller
// *************************************************************************************
//                          Dome Control
// =====================================================================================
//============ Dome Spin Signals ============
/* 2 options for control
 * Opt A) = "X" or "O" to spin left or right at set speed
 * Opt 2] = L3+Joystick to control dome speed rotatio
 */
int ps3DomeDrive(PS3BT* myPS3 = PS3Nav) {
  int domeRotationSpeed = 0;
  if (myPS3->getButtonPress(L1)&&!(myPS3->getButtonPress(L2)&&myPS3->getButtonPress(L3))) {              // L1 modifier
    if (myPS3->getButtonPress(CROSS)) {
      domeRotationSpeed = -75;
      #ifdef DEBUG_ON
        Serial.print(F("Dome Spin Command from Cross.  Spinning Dome at: "));
        Serial.println(domeRotationSpeed);
      #endif
    } else if (myPS3->getButtonPress(CIRCLE)) {
      domeRotationSpeed = 75;
      #ifdef DEBUG_ON
        Serial.print(F("Dome Spin Command from Circle.  Spinning Dome at: "));
        Serial.println(domeRotationSpeed);
      #endif
    } 
  } else if (myPS3->getButtonPress(L3)&&!myPS3->getButtonPress(L1)&&!myPS3->getButtonPress(L2)) {       // L3 Modifier
    int joystickPosition = myPS3->getAnalogHat(LeftHatX);
    domeRotationSpeed = (map(joystickPosition, 0, 255, -speedData[3], speedData[3]));
    #ifdef DEBUG_ON
        Serial.print(F("Dome Spin Command from L3+Joystick.  Spinning Dome at: "));
        Serial.println(domeRotationSpeed);
      #endif
    if ( abs(joystickPosition - 128) < joyStickData[1] ) 
      domeRotationSpeed = 0;
  }
  return domeRotationSpeed;
}

//============ SyRen Control ============
void rotateDome(int domeRotationSpeed, String mesg)
{
  /* Constantly sending commands to the SyRen (Dome) is causing foot motor delay.
   * Lets reduce that chatter by trying 3 things:
   *  1.) Eliminate a constant stream of "don't spin" messages (isMotorStopped[1] flag)
   *  2.) Add a delay between commands sent to the SyRen (prevMillis[1] timer)
   *  3.) Switch to real UART on the MEGA (Likely the *CORE* issue and solution)
   *  4.) Reduce the time-out of the SyRen - just better for safety!
   */
  currentMillis = millis();
  if ( (!isMotorStopped[1] || domeRotationSpeed != 0) && ((currentMillis - prevMillis[1]) > (2*serialLatency) )  ){
    #ifdef SHADOW_VERBOSE      
      output += "DEBUG:  Dome Rotation called by: ";
      output += mesg;
      if (domeRotationSpeed < 0){
        output += "  Spinning Dome Left at speed: "; 
      } else if (domeRotationSpeed > 0){
        output += "  Spinning Dome Right at speed: "; 
      } else {
        output += "  Stopping Dome Spin speed: "; 
      }    
      output += domeRotationSpeed; 
      output += "\r\n";
    #endif
    if (domeRotationSpeed != 0) {
      isMotorStopped[1] = false;
    } else {
      isMotorStopped[1] = true;
    }
    prevMillis[1] = currentMillis;      
    SyR->motor(domeRotationSpeed);
  }
}

//============ Value Control ============
void domeDrive()
{
  //Flood control prevention
  //This is intentionally set to double the rate of the Foot Motor Latency
  if ((millis() - prevMillis[1]) < (2*serialLatency) ) return;  

  int domeRotationSpeed = 0;
  int ps3NavControlSpeed = 0;
  if (PS3Nav->PS3NavigationConnected) ps3NavControlSpeed = ps3DomeDrive(PS3Nav);
  domeRotationSpeed = ps3NavControlSpeed; 
  rotateDome(domeRotationSpeed,"Controller Move");
}

//============ Data Display Controls ============
void ps3MsgSelect(PS3BT* myPS3 = PS3Nav){
  if (myPS3->getButtonClick(PS)) {
    if(msgData >= maxMsg){
      msgData = 0;
    }
    if (msgData < maxMsg){
      msgData++;
    }
    #ifdef DEBUG_ON
      Serial.print(F("Set Dome Data Port Message to: "));
      Serial.println(msgData);
    #endif
    String tempData = "Dome Msg: ";
    lcdMsg = tempData + msgData;
    lcdPrint(lcdMsg);
    Serial3.print(msgData);
  }
}

void sendMsgData(){
  if (PS3Nav->PS3NavigationConnected) ps3MsgSelect(PS3Nav);
}

// =====================================================================================
//                          END Dome Control
// *************************************************************************************
// /////////////////////////Process PS3 Controller Fault Detection//////////////////////
// =====================================================================================
//TODO:  bool criticalFaultDetect(PS3BT* myPS3 = PS3Nav, int controllerNumber = 1)
bool criticalFaultDetect()
{
    if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
    {
        lastMsgTime = PS3Nav->getLastMessageTime();
        currentTime = millis();
        if ( currentTime >= lastMsgTime)
        {
          msgLagTime = currentTime - lastMsgTime;
        } else
        {
             #ifdef DEBUG_ON
               output += "Waiting for PS3Nav Controller Data\r\n";
             #endif
             badPS3Data++;
             msgLagTime = 0;
        }
        
        if (msgLagTime > 100 && !isMotorStopped[0])
        {
            #ifdef DEBUG_ON
              output += "It has been 100ms since we heard from the PS3 Controller\r\n";
              output += "Shut downing motors, and watching for a new PS3 message\r\n";
            #endif
            stopFeet();
            SyR->stop();
            isMotorStopped[0] = true;
            return true;
        }
        if ( msgLagTime > 30000 )
        {
            #ifdef DEBUG_ON
              output += "It has been 30s since we heard from the PS3 Controller\r\n";
              output += "msgLagTime:";
              output += msgLagTime;
              output += "  lastMsgTime:";
              output += lastMsgTime;
              output += "  millis:";
              output += millis();            
              output += "\r\nDisconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }

        //Check PS3 Signal Data
        if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
        {
            // We don't have good data from the controller.
            //Wait 10ms, Update USB, and try again
            delay(10);
            Usb.Task();
            if(!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged))
            {
                badPS3Data++;
                #ifdef DEBUG_ON
                    output += "\r\nInvalid data from PS3 Controller.";
                #endif
                return true;
            }
        }
        else if (badPS3Data > 0)
        {
            //output += "\r\nPS3 Controller  - Recovered from noisy connection after: ";
            //output += badPS3Data;
            badPS3Data = 0;
        }
        if ( badPS3Data > 10 )
        {
            #ifdef DEBUG_ON
                output += "Too much bad data coming from the PS3 Controller\r\n";
                output += "Disconnecting the controller.\r\n";
            #endif
            PS3Nav->disconnect();
        }
    }
    else if (!isMotorStopped[0])
    {
        #ifdef DEBUG_ON      
            output += "No Connected Controllers were found\r\n";
            output += "Shuting downing motors, and watching for a new PS3 message\r\n";
        #endif
        stopFeet();
        SyR->stop();
        isMotorStopped[0] = true;
        return true;
    }
    return false;
}
// =====================================================================================
// /////////////////////////END of PS3 Controller Fault Detection///////////////////////
// *************************************************************************************
//                          Motor Drive Control
// =====================================================================================
// quick function to stop the feet depending on which drive system we're using...
void stopFeet() {
  ST->stop();
}

bool ps3FootMotorDrive(PS3BT* myPS3 = PS3Nav)
{
  speedData[5] = 0;
  int stickSpeed = 0;
  int turnnum = 0;

  if (isPS3NavigatonInitialized)
  {
      // Additional fault control.  Do NOT send additional commands to Sabertooth if no controllers have initialized.
      if (!isStickEnabled)
      {
            #ifdef SHADOW_VERBOSE
              if ( abs(myPS3->getAnalogHat(LeftHatY)-128) > joyStickData[0])
              {
                output += "Drive Stick is disabled\r\n";
              }
            #endif
          stopFeet();
          isMotorStopped[0] = true;
      } else if (!myPS3->PS3NavigationConnected)
      {
          stopFeet();
          isMotorStopped[0] = true;
      } else if ( myPS3->getButtonPress(L1) )
      {
          //TODO:  Does this need to change this when we support dual controller, or covered by improved isStickEnabled
          stopFeet();
          isMotorStopped[0] = true;
      } else
      {
          //make those feet move!!!///////////////////////////////////////////////////
          int joystickPosition = myPS3->getAnalogHat(LeftHatY);
          isMotorStopped[0] = false;
          #if FOOT_CONTROLLER == 0
          if (myPS3->getButtonPress(L2))
          {
            int throttle = 0;
            if (joystickPosition < 127)
            {
                throttle = joystickPosition - myPS3->getAnalogButton(L2);
            } else
            {
                throttle = joystickPosition + myPS3->getAnalogButton(L2);
            }
            stickSpeed = (map(throttle, -255, 510, -speedData[1], speedData[1]));                
          } else 
          {
            stickSpeed = (map(joystickPosition, 0, 255, -speedData[0], speedData[0]));
          }          

          if ( abs(joystickPosition-128) < joyStickData[0])
          {
              speedData[5] = 0;
          } else if (speedData[5] < stickSpeed)
          {
              if (stickSpeed-speedData[5]<(speedData[4]+1))
                  speedData[5]+=speedData[4];
              else
                  speedData[5] = stickSpeed;
          }
          else if (speedData[5] > stickSpeed)
          {
              if (speedData[5]-stickSpeed<(speedData[4]+1))
                  speedData[5]-=speedData[4];
              else
                  speedData[5] = stickSpeed;  
          }
          
          turnnum = (myPS3->getAnalogHat(LeftHatX));

          //TODO:  Is there a better algorithm here?  
          if ( abs(speedData[5]) > 50)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(speedData[2]/4), (speedData[2]/4)));
          else if (turnnum <= 200 && turnnum >= 54)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(speedData[2]/3), (speedData[2]/3)));
          else if (turnnum > 200)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 201, 255, speedData[2]/3, speedData[2]));
          else if (turnnum < 54)
              turnnum = (map(myPS3->getAnalogHat(LeftHatX), 0, 53, -speedData[2], -(speedData[2]/3)));
          #endif

          currentMillis = millis();
          if ( (currentMillis - prevMillis[0]) > serialLatency  )
          {

          #ifdef SHADOW_VERBOSE      
          if ( speedData[5] < -joyStickData[2] || speedData[5] > joyStickData[2])
          {
            output += "Driving Droid at footSpeed: ";
            output += speedData[5];
            output += "!  DriveStick is Enabled\r\n";
            output += "Joystick: ";              
            output += myPS3->getAnalogHat(LeftHatX);
            output += "/";              
            output += myPS3->getAnalogHat(LeftHatY);
            output += " turnnum: ";              
            output += turnnum;
            output += "/";              
            output += speedData[5];
            output += " Time of command: ";              
            output += millis();
          }
          #endif

          ST->turn(turnnum * invertTurnDirection);
          ST->drive(speedData[5]);
          // The Sabertooth won't act on mixed mode packet serial commands until
          // it has received power levels for BOTH throttle and turning, since it
          // mixes the two together to get diff-drive power levels for both motors.
           prevMillis[0] = currentMillis;
          return true; //we sent a foot command   
          }
      }
  }
  return false;
}

void footMotorDrive()
{
  //Flood control prevention
  if ((millis() - prevMillis[0]) < serialLatency) return;  
  if (PS3Nav->PS3NavigationConnected) ps3FootMotorDrive(PS3Nav);
}  
// =====================================================================================
//                          END Motor Drive Control
// *************************************************************************************
//                          Body Control
// =====================================================================================

//============  Utility Arms  ============
void ps3utilityArms(PS3BT* myPS3 = PS3Nav) {
  if (!myPS3->getButtonPress(L1)&&!myPS3->getButtonPress(L2)&&!myPS3->getButtonPress(L3)) {              // No modifier
    if(myPS3->getButtonClick(CROSS)) {
      #ifdef DEBUG_ON
        Serial.println(F("Opening/Closing top utility arm"));
      #endif
      if (isUtilArmOpen[0] == false){
        lcdMsg = "Top Util Arm Open";
      } else {
        lcdMsg = "Top Util Arm Close";
      }
      lcdPrint(lcdMsg);
      waveUtilArm(UTILITY_ARM_TOP);
    }
    if(myPS3->getButtonClick(CIRCLE)) {
      #ifdef DEBUG_ON
        Serial.println(F("Opening/Closing bottom utility arm"));
      #endif
      if (isUtilArmOpen[1] == false){
        lcdMsg = "Btm Util Arm Open";
      } else {
        lcdMsg = "Btm Util Arm Close";
      }
      lcdPrint(lcdMsg);
      waveUtilArm(UTILITY_ARM_BTM);
    }
  }
}

void utilityArms() {
  if (PS3Nav->PS3NavigationConnected) ps3utilityArms(PS3Nav);
}

void waveUtilArm(int arm) {
  switch (arm) {
    case UTILITY_ARM_TOP:
      if(isUtilArmOpen[0] == false){
        openUtilArm(UTILITY_ARM_TOP);
      } else {
        closeUtilArm(UTILITY_ARM_TOP);
      }
      break;
    case UTILITY_ARM_BTM:  
      if(isUtilArmOpen[1] == false){
        openUtilArm(UTILITY_ARM_BTM);
      } else {
        closeUtilArm(UTILITY_ARM_BTM);
      }
      break;
  }
}

void openUtilArm(int arm) {moveUtilArm(arm, utilArmMinMax[1]);}

void closeUtilArm(int arm) {moveUtilArm(arm, utilArmMinMax[0]);}

void moveUtilArm(int arm, int position) {
  switch (arm) {
    case UTILITY_ARM_TOP:
      UtilArmTopServo.write(position);
      if ( position == utilArmMinMax[0]) {
        isUtilArmOpen[0] = false;
      } else {
        isUtilArmOpen[0] = true;
      }
      break;
    case UTILITY_ARM_BTM:  
      UtilArmBottomServo.write(position);
      if ( position == utilArmMinMax[0]) {
        isUtilArmOpen[1] = false;
      } else {
        isUtilArmOpen[1] = true;
      }
      break;
  }
}

//============ Coin Slot Flash ============
void flashCoinSlotLEDs(){
  for(int i = 0; i<numberOfCoinSlotLEDs; i++) {
    if(millis() > nextCoinSlotLedFlash[i]) {
      if(coinSlotLedState[i] == LOW) coinSlotLedState[i] = HIGH; 
      else coinSlotLedState[i] = LOW;
      digitalWrite(COIN_SLOT_LED_PINS[i],coinSlotLedState[i]);
      nextCoinSlotLedFlash[i] = millis()+random(100, 1000) ; // next toggle random time
    } 
  }
}

//============  Settings Toggle Control  ============
void ps3ToggleSettings(PS3BT* myPS3 = PS3Nav) {
  // --------- Disconnect Controller ---------
  if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(L3)) {
    #ifdef DEBUG_ON
      Serial.println(F("Disconnecting the controller."));
    #endif
    myPS3->disconnect();
  }

  // --------- Disable Joystick ---------
  if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CROSS)) {
    #ifdef DEBUG_ON
      Serial.println(F("Disabling the DriveStick."));
    #endif
    isStickEnabled = false;
    lcdMsg = "Joystick DISABLED";
    lcdPrint(lcdMsg);
  }
  
  // --------- Enable Joystick ---------
  if(myPS3->getButtonPress(PS)&&myPS3->getButtonClick(CIRCLE)) {
    #ifdef DEBUG_ON
      Serial.println(F("Enabling the DriveStick"));
    #endif
    isStickEnabled = true;
    lcdMsg = "Joystick ENABLED";
    lcdPrint(lcdMsg);
  }

  // --------- Disable Dome Automation ---------
  if(myPS3->getButtonPress(L2)&&myPS3->getButtonClick(CROSS)) {
    if(isAutomateDomeOn) {
      #ifdef DEBUG_ON
        Serial.println(F("Disabling the Dome Automation"));
      #endif
      isAutomateDomeOn = false;
      domeStatus = 0;
      domeTargetPosition = 0;
      SyR->stop();
      action = 0;
      lcdMsg = "Dome Auto OFF";
      lcdPrint(lcdMsg);
    }
  }

  // --------- Enable Dome Automation ---------
  if(myPS3->getButtonPress(L2)&&myPS3->getButtonClick(CIRCLE)) {
    #ifdef DEBUG_ON
      Serial.println(F("Enabling the Dome Automation"));
    #endif
    lcdMsg = "Dome Auto ON";
    lcdPrint(lcdMsg);
    isAutomateDomeOn = true;
  }
}

void toggleSettings() {
  if (PS3Nav->PS3NavigationConnected) ps3ToggleSettings(PS3Nav);
}

// =====================================================================================
//                          END Body Control
// *************************************************************************************
//                          Sound Control
// =====================================================================================

void ps3SndCtrl(PS3BT* myPS3 = PS3Nav) {
  if (!myPS3->getButtonPress(L1)&&!myPS3->getButtonPress(L2)) {                                      // No modifier
    if (myPS3->getButtonClick(UP)){
      #ifdef DEBUG_ON
        Serial.println(F("UP")); 
      #endif
      lcdMsg = "UP";
      lcdPrint(lcdMsg);
      #ifdef DEBUG_ON
        Serial.println(lcdMsg); 
      #endif
      sndData = 1;
    } else if (myPS3->getButtonClick(RIGHT)){
      #ifdef DEBUG_ON
        Serial.println(F("RT"));
      #endif
      lcdMsg = "RIGHT";
      lcdPrint(lcdMsg);
      #ifdef DEBUG_ON
        Serial.println(lcdMsg); 
      #endif
      sndData = 2;
    } else if (myPS3->getButtonClick(DOWN)){
      #ifdef DEBUG_ON
        Serial.println(F("DN"));
      #endif
      lcdMsg = "DOWN";
      lcdPrint(lcdMsg);
      #ifdef DEBUG_ON
        Serial.println(lcdMsg); 
      #endif
      sndData = 3;
    } else if (myPS3->getButtonClick(LEFT)){
      #ifdef DEBUG_ON
        Serial.println(F("LT"));
      #endif
      lcdMsg = "LEFT";
      lcdPrint(lcdMsg);
      #ifdef DEBUG_ON
        Serial.println(lcdMsg); 
      #endif
      sndData = 4;
    }
  } else if (myPS3->getButtonPress(L1)&&!myPS3->getButtonPress(L2)) {                                // L1 modifier
    if (myPS3->getButtonClick(UP)){
      #ifdef DEBUG_ON
        Serial.println(F("UP+L1"));
      #endif
      lcdMsg = "L1+LEFT";
      lcdPrint(lcdMsg);
      sndData = 5;
    } else if (myPS3->getButtonClick(RIGHT)){
      #ifdef DEBUG_ON
        Serial.println(F("RT+L1"));
      #endif
      lcdMsg = "L1+RIGHT";
      lcdPrint(lcdMsg);
      sndData = 6;
    } else if (myPS3->getButtonClick(DOWN)){
      #ifdef DEBUG_ON
        Serial.println(F("DN+L1"));
      #endif
      lcdMsg = "L1+DOWN";
      lcdPrint(lcdMsg);
      sndData = 7;
    } else if (myPS3->getButtonClick(LEFT)){
      #ifdef DEBUG_ON
        Serial.println(F("LT+L1"));
      #endif
      lcdMsg = "L1+LEFT";
      lcdPrint(lcdMsg);
      sndData = 8;
    }
  } else if (!myPS3->getButtonPress(L1)&&myPS3->getButtonPress(L2)) {                                // L2 modifier
    if (myPS3->getButtonClick(UP)){
      #ifdef DEBUG_ON
        Serial.println(F("UP+L2"));
      #endif
      lcdMsg = "L2+UP";
      lcdPrint(lcdMsg);
      sndData = 9;
    } else if (myPS3->getButtonClick(RIGHT)){
      #ifdef DEBUG_ON
        Serial.println(F("RT+L2"));
      #endif
      lcdMsg = "L2+RIGHT";
      lcdPrint(lcdMsg);
      sndData = 10;
    } else if (myPS3->getButtonClick(DOWN)){
      #ifdef DEBUG_ON
        Serial.println(F("DN+L2"));
      #endif
      lcdMsg = "L2+DOWN";
      lcdPrint(lcdMsg);
      sndData = 11;
    } else if (myPS3->getButtonClick(LEFT)){
      #ifdef DEBUG_ON
        Serial.println(F("LT+L2"));
      #endif
      lcdMsg = "L2+LEFT";
      lcdPrint(lcdMsg);
      sndData = 12;
    }
  } else if (myPS3->getButtonPress(L1)&&myPS3->getButtonPress(L2)) {                                 // L1 & L2 modifier
    if (myPS3->getButtonClick(UP)){
      #ifdef DEBUG_ON
        Serial.println(F("UP+L1+L2"));
      #endif
      lcdMsg = "L1+L2+UP";
      lcdPrint(lcdMsg);
      sndData = 13;
    } else if (myPS3->getButtonClick(RIGHT)){
      #ifdef DEBUG_ON
        Serial.println(F("RT+L1+L2"));
      #endif
      lcdMsg = "L1+L2+RIGHT";
      lcdPrint(lcdMsg);
      sndData = 14;
    } else if (myPS3->getButtonClick(DOWN)){
      #ifdef DEBUG_ON
        Serial.println(F("DN+L1+L2"));
      #endif
      lcdMsg = "L1+L2+DOWN";
      lcdPrint(lcdMsg);
      sndData = 15;
    } else if (myPS3->getButtonClick(LEFT)){
      #ifdef DEBUG_ON
        Serial.println(F("LT+L1+L2"));
      #endif
      lcdMsg = "L1+L2+LEFT";
      lcdPrint(lcdMsg);
      sndData = 16;
    }
  }
  if (sndData > 0){
    #ifdef DEBUG_ON
      Serial.print(F("Sound Data set to: ")); 
      Serial.println(sndData);
      Serial.println(F("Transmitting Sound Data")); 
    #endif
    Serial1.write(sndData);                                                                       // Transmit selection to Sound Processor
    sndData = 0;
    #ifdef DEBUG_ON
      Serial.print(F("Sound Data set to: ")); 
      Serial.println(sndData);
    #endif
  } 
}

void sndCtrl(){
  if (PS3Nav->PS3NavigationConnected) ps3SndCtrl(PS3Nav);
}

// =======================================================================================
//                          END Sound Control
// *************************************************************************************
//                          Controller Test Function
// =======================================================================================
#ifdef TEST_CONROLLER
void testPS3Controller(PS3BT* myPS3 = PS3Nav)
{
    if (myPS3->PS3Connected || myPS3->PS3NavigationConnected) {
        if (myPS3->getButtonPress(PS) && (myPS3->getAnalogHat(LeftHatX) > 137 || myPS3->getAnalogHat(LeftHatX) < 117 || myPS3->getAnalogHat(LeftHatY) > 137 || myPS3->getAnalogHat(LeftHatY) < 117) {     
            output += "LeftHatX: ";
            output += myPS3->getAnalogHat(LeftHatX);
            output += "\tLeftHatY: ";
            output += myPS3->getAnalogHat(LeftHatY);
        }
        //Analog button values can be read from almost all buttons
        if (myPS3->getButtonPress(PS) && (myPS3->getAnalogButton(L2)))
        {
            if (output != "")
                output += "\r\n";
            output += "L2: ";
            output += myPS3->getAnalogButton(L2);
        }
        if (myPS3->getButtonClick(L2)) {
            output += " - L2";
            //myPS3->disconnect();
        }
        if (output != "") {
            Serial.println(output);
            if (SerialBT.connected)
                SerialBT.println(output);
            output = ""; // Reset output string
        }
        if (myPS3->getButtonClick(PS)) {
            output += " - PS";
            //myPS3->disconnect();
        }
        else {
            if (myPS3->getButtonClick(CIRCLE))
                output += " - Circle";
            if (myPS3->getButtonClick(CROSS))
                output += " - Cross";
            if (myPS3->getButtonClick(UP)) {
                output += " - Up";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED4);
                }
            }
            if (myPS3->getButtonClick(RIGHT)) {
                output += " - Right";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED1);
                }
            }
            if (myPS3->getButtonClick(DOWN)) {
                output += " - Down";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED2);
                }
            }
            if (myPS3->getButtonClick(LEFT)) {
                output += " - Left";
                if (myPS3->PS3Connected) {
                    myPS3->setLedOff();
                    myPS3->setLedOn(LED3);
                }
            }
            if (myPS3->getButtonClick(L1))
                output += " - L1";
            if (myPS3->getButtonClick(L3))
                output += " - L3";
        }
    }          
}
#endif
// =======================================================================================
//                          END Controller Test Function
// *************************************************************************************
//                          LCD Print Function
// =======================================================================================
void lcdPrint(String& lcdMsg) {
  #ifdef DEBUG_ON
    Serial.println(F("LCD Update Called"));
    Serial.print(F("LCD Message: "));
    Serial.println(lcdMsg); 
  #endif
  lcdPrevMsg[0] = lcdPrevMsg[1];
  #ifdef DEBUG_ON
    Serial.print(F("Prev Message 0 set to: "));
    Serial.println(lcdPrevMsg[0]); 
  #endif
  lcdPrevMsg[1] = lcdPrevMsg[2];
  #ifdef DEBUG_ON
    Serial.print(F("Prev Message 1 set to: "));
    Serial.println(lcdPrevMsg[1]); 
  #endif
  lcdPrevMsg[2] = lcdPrevMsg[3];
  #ifdef DEBUG_ON
    Serial.print(F("Prev Message 2 set to: "));
    Serial.println(lcdPrevMsg[2]); 
  #endif
  lcdPrevMsg[3] = lcdMsg;
  #ifdef DEBUG_ON
    Serial.print(F("Prev Message 3 set to: "));
    Serial.println(lcdPrevMsg[3]); 
  #endif
  lcd.clear();
  for (int i = 0; i <= 3; i++){
    lcd.setCursor(0,i);
    lcdMsg = lcdPrevMsg[i];
    lcd.print(lcdMsg);
    Serial.println(lcdPrevMsg[i]);
  }
  /*
  if (lcdRow < 3) {
    lcdRow++;
  } else {
    lcdRow = 0;
  }
  lcd.setCursor(0,lcdRow);
  lcd.print("                    ");
  lcd.setCursor(0,lcdRow);
  lcd.print(lcdMsg);
  */
}

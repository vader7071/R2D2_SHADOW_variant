/*
* Arduino Leonardo
* 
* 2019-12-24
* 
* R2 Sound Controller
* Uses:
*   (1) Adafruit Music Maker Shield (PID 1788)
*   (1) Adafruit 20W Amplifier (PID 1752)
* 
* Body Arduino Mega communicates to this unit via serial comm.
* 
* Serial Comm Port is pins 12 & 13 on this processor   
* 
*/

#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7       // VS1053 chip select pin (output)
#define SHIELD_DCS    6       // VS1053 Data/command select pin (output)
#define CARDCS 4              // Card chip select pin
#define DREQ 3                // VS1053 Data request, DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
//#define DEBUG_ON

#define MAX9744_I2CADDR 0x4B

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);
SoftwareSerial comPort(8,9);                                          // Create second serial port.  Pin 8 = RX, 9 = TX

const int r2SoundFiles[] = {"R2001.mp3","R2002.mp3","R2003.mp3","R2004.mp3","R2005.mp3","R2006.mp3","R2007.mp3","R2008.mp3","R2009.mp3","R2010.mp3","R2011.mp3","R2012.mp3","R2013.mp3","R2014.mp3","R2015.mp3","R2016.mp3","R2017.mp3","R2018.mp3","R2019.mp3","R2020.mp3","R2021.mp3","R2022.mp3","R2023.mp3","R2024.mp3","R2025.mp3","R2026.mp3","R2027.mp3","R2028.mp3","R2029.mp3","R2030.mp3"};
const int dpSoundFiles[] = {"DP001.mp3","DP002.mp3","DP003.mp3","DP004.mp3","DP005.mp3","DP006.mp3","DP007.mp3","DP008.mp3","DP009.mp3","DP010.mp3","DP011.mp3"};

// Make sure the PG13 and "clean" sound files are listed first to make the array below work.  This allows for a PG13 interaction and a Rated-R interaction.

const byte sndList[3] = {30,11,5};  // [0] = Qty of R2. [1] = *ALL* DP sounds, [2] = PG13 sound files

byte sndFile = 0;
byte volume = 95;    // 0 to 100
uint8_t sndData = 0;

//============  Setup function  ============
void setup() {
  randomSeed(analogRead(A0));
  Serial.begin(9600);
  comPort.begin(9600);
  #ifdef DEBUG_ON
    Serial.print(F("\r\nSound Control Comm Port Open"));
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif

//#if !defined(__MIPSEL__)
//  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//#endif

  // -----
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while(1);
  }
  Serial.println(F("VS1053 found"));
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);
  }
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)){
    Serial.println(F("DREQ pin is not an interrupt pin"));
    while (1);
  }
  #ifdef DEBUG_ON
    printDirectory(SD.open("/"), 0);
    Serial.println(F("Playing Startup track"));
  #endif
  musicPlayer.playFullFile("/startup2.mp3");

  Wire.begin();

  setVolume(volume);
}

//============  Loop Function  ============
void loop() {
  if(comPort.available() > 0){
    sndData = comPort.read();
    #ifdef DEBUG_ON
      Serial.print(F("Data received from Primary Processor: "));
      Serial.println(sndData);
    #endif
  }
  if (sndData > 0){ 
    sndCtrl();
  }
}

//============  Sound Control  ============
void sndCtrl(){
  switch (sndData) {
    case 0:                                     // No data received
      return;
    case 1:                                     // Up, no modifier
      musicPlayer.playFullFile("DP001.mp3");
      sndData = 0;
      break;
    case 2:                                     // Right, no modifier
      musicPlayer.playFullFile("DP002.mp3");
      sndData = 0;
      break;
    case 3:                                     // Down, no modifier
      musicPlayer.playFullFile("DP003.mp3");
      sndData = 0;
      break;
    case 4:                                     // Left, no modifier
      musicPlayer.playFullFile("DP004.mp3");
      sndData = 0;
      break;
    case 5:                                     // Up + L1
      musicPlayer.playFullFile("DP005.mp3");
      sndData = 0;
      break;
    case 6:                                     // Right + L1
      musicPlayer.playFullFile("DP006.mp3");
      sndData = 0;
      break;
    case 7:                                     // Down + L1
      musicPlayer.playFullFile("DP007.mp3");
      sndData = 0;
      break;
    case 8:                                     // Left + L1
      musicPlayer.playFullFile("DP008.mp3");
      sndData = 0;
      break;
    case 9:                                     // Up + L2
      musicPlayer.playFullFile("R2001.mp3");
      sndData = 0;
      break;
    case 10:                                    // Right + L2
      musicPlayer.playFullFile("R2002.mp3");
      sndData = 0;
      break;
    case 11:                                    // Down + L2
      musicPlayer.playFullFile("R2003.mp3");
      sndData = 0;
      break;
    case 12:                                    // Left + L2
      musicPlayer.playFullFile("R2004.mp3");
      sndData = 0;
      break;
    case 13:                                    // Up + L1 + L2
      if (volume >= 100) {
        volume = 100;
        sndErr();
      } else {
        volume = volume + 5;
        setVolume(volume);
        musicPlayer.playFullFile("/volup.mp3");
      }
      sndData = 0;
      break;
    case 14:                                    // Right + L1 + L2
      r2Random();
      sndData = 0;
      break;
    case 15:                                    // Down + L1 + L2
      if (volume <= 0) {
        volume = 0;
        sndErr();
      } else {
        volume = volume - 5;
        setVolume(volume);
        musicPlayer.playFullFile("/voldn.mp3");
      }
      sndData = 0;
      break;
    case 16:                                    // Left + L1 + L2
      dpRandom();
      sndData = 0;
      break;
    case 17:                                    // TBD
      dpCleanRandom();
      sndData = 0;
      break;
    default:
      break;
  }
}

//============  File listing helper  ============
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

//============  Set Volume  ============
void setVolume(byte v) {
  byte vol1752 = map(v,0,100,0,63);   // Amplifier 0% = 0, 100% = 63
  byte vol1788 = map(v,0,100,255,0);  // Music Maker 0% = 255, 100% = 0
  
  #ifdef DEBUG_ON
    Serial.print("Setting volume to ");
    Serial.print(v);
    Serial.println("%");
  //1752 20W Amplifier
    Serial.print("1752: ");
    Serial.println(vol1752);
  #endif
  Wire.beginTransmission(MAX9744_I2CADDR);
  Wire.write(vol1752);
  //1788 Music Maker
  #ifdef DEBUG_ON
    Serial.print("1788: ");
    Serial.println(vol1788);
  #endif
  musicPlayer.setVolume(vol1788,vol1788);
}

//============  R2 Random Sound  ============
void r2Random() {
  sndFile = random(sndList[0]);
  musicPlayer.playFullFile(r2SoundFiles[sndFile]);
}

//============  DP Random Sound  ============
void dpRandom() {
  sndFile = random(sndList[1]);
  musicPlayer.playFullFile(dpSoundFiles[sndFile]);
}

//============  DP Clean Random Sound  ============
void dpCleanRandom() {
  sndFile = random(sndList[2]);
  musicPlayer.playFullFile(dpSoundFiles[sndFile]);
}

//============  Volume Set Error  ============
void sndErr() {
  setVolume(95);
  musicPlayer.playFullFile("/err.mp3");
  setVolume(volume);
}

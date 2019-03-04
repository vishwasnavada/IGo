
/* This program was created for Sony Spresense Board to log the data of OBD II port and road damage detection using IMU MPU6050
    This program is created under open source category for non commercial use
    OBD II data is logged over bluetooth through HC 05 Bluetooth Module
    This code looks big because we have to read all the data from the OBD II port by querying different AT Commands for different vehicle parameters
*/
#define VER_SPRESENSE                            // Compile for SPRESENSE or UNO
//#define VER_MONITORING                          // If defined send some debuging messages over IDE monitor
//#define VER_AUDIO                               // If defined incorporates the Audio component
#define VER_OBD                                 // If not defined will run simulating OBD input (Check SD>file>KML) 


#include<Wire.h>
#include <GNSS.h>                               // Include the Sony SPRESENSE GPS library
SpGnss gnss;                                    // Set the alias to the library functions
#include <SPI.h>                                // Supports SD card interface
//#include <SD.h>                                 // Include the Sony SPRESENSE SD card library
#define SDCARD_SS_PIN A3                        // Define SD card Slave Select on A3
#include <SDHCI.h>
SDClass SD;                             // Use Serial Process Interface (SPI) 4
#ifdef VER_AUDIO
#include <Audio.h>                            // Include the Sony SPRESENSE Audio library
#include <fcntl.h>
AudioClass myAudio;                          // Instanciate the Audio class
#endif

#define PINRX       0                           // D0
#define PINTX       1                           // D1 

char rxData[32];                                // Character buffer for OBD data recieved. Assumes short bursts
char rxIndex = 0;                               // Array index
char buf[10];                                   // for fprint ourput
int vehicleSpeed = 0;                           // Vehicle Speed
int lastSpeed = 0;                              // Last Vehicle Speed - Used on Audio
int vehicleRPM = 0;                             // Vehicle RPMs
unsigned long markTime;                         // Time mark for the delay timer
unsigned long markTime2;                        // Time mark for the OBD timeout
bool timeoutOBD = false;                        // Timeout on OBD response
int LastPrintMin = 0;                           // Marker for GNSS 1 minute check
SpNavData *pNavData;                            // Pointer for NavData
String sdString;                                // Character Array to capture data and write to SD card
int caLen;                                      // Character Array Length
char caString[36];                              // Character Array for formating String
String fileName;                                // Name of the file to use
File myGNSSFile;                                // File handle for GNSS KML file
#ifdef VER_AUDIO
File myMP3File;                               // File handle for MP3 Audio file
#endif
File damageFile;                                //File to record road damage detection using IMU
bool sdCardOK = true;                           // Have to initialize SD card, will let error set to false
bool switchOn = true;                           // State of operation for SD file
bool fileClosed = true;                         // State of last processed file
bool fileStart = false;                         // Inication that file open an header written
int id = 0;

#ifndef VER_OBD
int tmpCnt = 0;                               // Used to debug standing alone (OBD not connected)
#endif

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
void setup() {
  //  timeoutOBD = false;                   // Not needed? Used when debuging memory issue
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);


  while (!Serial) {
    ; // wait for COM port to open/connect; USB Monitoring
  }
  waitTime(1000);
  Serial.println("IDE Monitoring: OK");

#ifdef VER_OBD
  Serial2.begin(9600);
  while (!Serial2) {
    ; // can be used to transmit data to internet through ESP 8266 or Node MCU board Serial port 0 or Serial.print is used by USB so we have to connect to Serial2 i.e 0 and 1
  }
#endif
  waitTime(1000);
  Serial.println("OBD-II UART: Is listenting");

  gnss.setDebugMode(PrintInfo);                              // Put GPS readings into a debug mode
  int r;
  r = gnss.begin();                                          // Initialize GNSS handling
  if (r != 0) {                                              // Error encountered
    Serial.print("GNSS: Unable to initialize! ("); Serial.print(r); Serial.println(")");
  } else {
    r = gnss.start(COLD_START);                              // Start GNSS listening
    if (r != 0) {                                            // Error encountered
      Serial.print("GNSS: Unable to start! ("); Serial.print(r); Serial.println(")");
    } else {
      Serial.println("GNSS: Is online/listening");           // GSNN is online
    }
  }

  if (SD.begin(SDCARD_SS_PIN)) {
    Serial.println("SD Card: Initialized!");
    sdCardOK = true;
  } else {
    Serial.println("SD Card: Initialization failed!");
    sdCardOK = false;
  }

  //Make arrangements to log the road damage detection .csv file
  File damageFile = SD.open("log.csv", FILE_WRITE);

  if (damageFile) {
    damageFile.println(", , , ,"); //Just a leading blank line, incase there was previous data

    String header = "ID, Time, Latitude, Longitude, Value"; //These will be the headers for your excel file, CHANGE "" to whatevr headers you would like to use
    damageFile.println(header);
    damageFile.close();
    Serial.println(header);
  }

#ifdef VER_AUDIO
  myAudio.begin();
  myAudio.setPlayerMode(AS_OUT_SP);
  myAudio.initPlayer(AS_INITPLAYER_MP3,
                     AS_INITPLAYER_INPUT_FS_44100,
                     AS_INITPLAYER_CHNL_STEREO);
  Serial.println("Audio setup complete");
#endif

#ifdef VER_AUDIO
  digitalWrite(13, HIGH);
#endif
#ifdef VER_OBD
  InitOBD();                                                // Initilize the connection
  ledOn(LED0);

#endif
}

boolean getNavData(SpNavData *pNavData) {
  boolean ret;
  if (pNavData->posDataExist == 0)  {
    Serial.println("GNSS: Position is not yet fixed");
    ret = false;
  } else {
    float fsec = (float)pNavData->time.sec + ((float)pNavData->time.usec / 1000000);
    ret = true;
  }
  return ret;
}

void loop() {

  if (digitalRead(5) == HIGH)
  {
    switchOn = true;
  }
  else
  {
    switchOn = false;
  }



  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  // Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  delay(333);





#ifndef VER_OBD
  tmpCnt++;
  Serial.print("Count: ");
  Serial.println(tmpCnt);
#endif



  // --- Get Vehicle Speed -----------------------------------------------------------------------------
#ifdef VER_OBD
  vehicleSpeed = getSpeed();                         // Get the Speed from the car via OBD interface

  sprintf(buf, "%03d", vehicleSpeed);                // Format reading

  waitTime(100);
#else
  vehicleSpeed = 36;                                 // Dummy up a vehicle Speed for non OBD run
#endif

#ifdef VER_AUDIO
  // --- Audible vehicle Speed -------------------------------------------------------------------------
  if ((vehicleSpeed >= 65) && (vehicleSpeed <= 125)) {          // Acceptible range
    if ((vehicleSpeed % 5) == 0) {                              // On a divisible by 5?
      if (vehicleSpeed != lastSpeed) {                          // We dont want to keep repeating ourselves!
        Serial.print("Audio: vehicleSpeed is "); Serial.println(vehicleSpeed);
        openPlay(vehicleSpeed);                                 // Play the Audio of the Vehicle speed
        lastSpeed = vehicleSpeed;                               // Reset the lastSpeed as not to repeat
      }
    }
  }
#endif

  // --- Get Vehicle RPMs ------------------------------------------------------------------------------
#ifdef VER_OBD
  vehicleRPM = getRPM();                                        // Get the RPMs from the car via OBD interface

  sprintf(buf, "%04d", vehicleRPM);                             // Format reading

  waitTime(100);
#else
  vehicleRPM = 1000;                                            // Dummy up a vehicle Speed for non OBD run
#endif

  // --- Get Vehicle Location --------------------------------------------------------------------------
  if (gnss.waitUpdate(-1)) {
    SpNavData NavData;
    gnss.getNavData(&NavData);

    if (getNavData(&NavData)) {                                 // Get updated position data if available
      ledOn(LED1);                                             // Indicate satillites acquired
      pNavData = &NavData;
      Serial.print("GNSS: Found Satellite! "); Serial.print(pNavData->longitude, 6); Serial.print(", "); Serial.print(pNavData->latitude, 6); Serial.print(", "); Serial.println(vehicleSpeed);
      caLen = sprintf(caString, "%3.6f,%3.6f,%d\0", pNavData->longitude, pNavData->latitude, (int)(vehicleSpeed + 100));
      sdString = caString;

      if (timeoutOBD == true) {                                 // Did we have a timeout? (Changed as "if(timeoutOBD){" did not work
        switchOn = false;                                       // Lost connection with OBD, drive to close the file22
        ledOff(LED0);                                          // Lost connection with OBD, kill status LED
        timeoutOBD = false;                                     // Reset timeout
      }

      // --- File handling only while GNSS data is available -----------------------------------------------------
      if (sdCardOK) {                                           // SD Card OK..
        if (switchOn) {                                         // Switch to record to file still ON...
          Serial.println("Switch is On");
          if (fileClosed) {                                     // File not open, then Open...
            caLen = sprintf(caString, "vl%02d%02d%02d.kml\0", pNavData->time.day, pNavData->time.hour, pNavData->time.minute);
            Serial.print("File="); Serial.print(caString); Serial.print("<"); Serial.print(caLen); Serial.println(">");
            fileName = caString;
            myGNSSFile = SD.open(fileName, FILE_WRITE);       // Format = "vl",dd=day,hh=hr,mm=min,".kml"
            if (myGNSSFile) {                                   // Opened OK
              Serial.println("SD Card: File opened!");
              ledOn(LED2);                                     // Logging begun
              fileClosed = false;                               // File is Open. No longer closed
              writeKMLheader();                                 // Write KML header
              fileStart = true;                                 // Indicate we have started file with header
            } else {
              Serial.println("SD Card: File Open error!");
              switchOn = false;
            }
          }
          if (!fileClosed) {
            if (myGNSSFile) {
              Serial.println("MyGNSSfile: File is open");
              if (fileStart) {                                  // Header already KML written
                fileStart = false;                              // Switch to KML details
                Serial.println("MyGNSSfile: Header will be written, switching to write coordinates");
              } else {                                           // With this we will miss 1st itteration, but OK
                writeKMLcoordinates(sdString);                  // Write KML details
                Serial.println("MyGNSSfile: Coordinates written");
              }
            } else {
              switchOn = false;                                 // Lost the file handle
              Serial.println("SD card: Lost the file handle or File error");
            }
          }
        } else {                                                 // Indication to close file
          Serial.println("Switch is Off");
          writeKMLfooter();                                     // Write KML footet
          myGNSSFile.flush();                                   // Push residule data from bus to file
          myGNSSFile.close();                                   // Close file
          ledOff(LED2);                                        // Logging ended
          fileClosed = true;                                    // file is closed
          waitTime(60000);                                      // Delay to give a subsequent opportunity to record
          switchOn = true;                                      // Reset the switch to start again
#ifdef VER_OBD
          InitOBD();                                            // Re-Initilize the connection
#endif
        }
      } else {
        Serial.println("SD Card: Failed to initialize in setup!");
      }

      String T = String(pNavData->time.day) + String(pNavData->time.hour) + String( pNavData->time.minute) + String( pNavData->time.sec);
      String datastring = String(id) + "," + String(T) + String(",") + String(pNavData->longitude, 6) + "," + String(pNavData->latitude, 6) + "," + String(AcZ);
      File damageFile = SD.open("log.csv", FILE_WRITE);

      if (damageFile) {
        damageFile.println(datastring);
        damageFile.close();
        Serial.println(datastring);
      }

    } else {
      Serial.println("GNSS: No satellite discovered");
      ledOff(LED1);                                          // Indicate satillites NOT acquired
    }
  } else {
    Serial.println("GNSS: Data not updated");                 // Catch if data not updated
  }
#ifndef VER_OBD
  if (tmpCnt > 900) {                                         // Terminated after 900 itterations (~12min)
    timeoutOBD = true;
    tmpCnt = 0;
  }
#endif

  id = id++;

}


// Write to SD KML Header

void writeKMLheader(void) {
  Serial.println("KML Header");
  sdString = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"; myGNSSFile.println(sdString);
  sdString = "<kml xmlns=\"http://www.opengis.net/kml/2.2\">"; myGNSSFile.println(sdString);
  sdString = "<Document>"; myGNSSFile.println(sdString);
  sdString = "<name>Paths</name>"; myGNSSFile.println(sdString);
  sdString = "<description>Vehicle pathing from OBD-II UART and Sony SPRESENSE. James Penner February 14, 2018</description>"; myGNSSFile.println(sdString);
  sdString = "<Style id=\"yellowLineGreenPoly\">"; myGNSSFile.println(sdString);
  sdString = "<LineStyle>"; myGNSSFile.println(sdString);
  sdString = "<color>7f00ffff</color>"; myGNSSFile.println(sdString);
  sdString = "<width>4</width>"; myGNSSFile.println(sdString);
  sdString = "</LineStyle>"; myGNSSFile.println(sdString);
  sdString = "<PolyStyle>"; myGNSSFile.println(sdString);
  sdString = "<color>7f00ff00</color>"; myGNSSFile.println(sdString);
  sdString = "</PolyStyle>"; myGNSSFile.println(sdString);
  sdString = "</Style>"; myGNSSFile.println(sdString);
  sdString = "<Placemark>"; myGNSSFile.println(sdString);
  sdString = "<name>Absolute Extruded</name>"; myGNSSFile.println(sdString);
  sdString = "<description>Transparent green wall with yellow outlines</description>"; myGNSSFile.println(sdString);
  sdString = "<styleUrl>#yellowLineGreenPoly</styleUrl>"; myGNSSFile.println(sdString);
  sdString = "<LineString>"; myGNSSFile.println(sdString);
  sdString = "<extrude>1</extrude>"; myGNSSFile.println(sdString);
  sdString = "<tessellate>1</tessellate>"; myGNSSFile.println(sdString);
  sdString = "<altitudeMode>absolute</altitudeMode>"; myGNSSFile.println(sdString);
  sdString = "<coordinates>"; myGNSSFile.println(sdString);
}


// Write to SD KML coordinates & Speed. Example: -112.2549277039738, 36.08117083492122, 75

void writeKMLcoordinates(String& sdString) {
  Serial.print("KML coordinates: ");
  Serial.println(sdString);
  myGNSSFile.println(sdString);
}


// Write to SD KML footer

void writeKMLfooter(void) {
  Serial.println("KML footer");
  sdString = "</coordinates>"; myGNSSFile.println(sdString);
  sdString = "</LineString>"; myGNSSFile.println(sdString);
  sdString = "</Placemark>"; myGNSSFile.println(sdString);
  sdString = "</Document>"; myGNSSFile.println(sdString);
  sdString = "</kml>"; myGNSSFile.println(sdString);
}


// Get vehicle speed km/h

int getSpeed(void) {
  Serial2.print("010D\r");                            // Get from OBD-II the --> Vehicle speed
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
#ifdef VER_MONITORING
  Serial.print("KMH:[010D], Receive:["); Serial.print(rxData);
  Serial.print("] Hex2DEC:["); Serial.print(strtol(&rxData[6], NULL, 16)); Serial.println("]");
#endif
  return strtol(&rxData[6], NULL, 16);                // "41 0D XX" --> Single byte value XX
}


// Get vehicle engine RPM

int getRPM(void) {
  Serial2.print("010C\r");                           // Get from OBD-II the --> Vehicle RPM
  waitTime(50);                                      // Wait for reposne to complete before reading
  readOBD();                                         // Read response from board
#ifdef VER_MONITORING
  Serial.print("RPM:[010C], Receive:["); Serial.print(rxData); Serial.println("]");
#endif
  // Calculation to RPM is based on the following:
  // - "41 0C XX YY" --> Two byte value XX YY
  // - ((high order XX byte *256)+low order YY byte) / 4
  // - RPM value is in 1/4 RPM, divide by 4 to get whole RPM
  return ((strtol(&rxData[6], NULL, 16) * 256) + strtol(&rxData[9], NULL, 16)) / 4;
}


// Get vehicle Oil Temperature in Celcius

int getOil(void) {
  Serial2.print("015C\r");                            // Get from OBD-II the --> Oil Temperature in Celcius
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return int(((strtol(&rxData[6], NULL, 16) * 9.0) / 5.0 + 32) - 40);  // "41 5C XX" --> Single byte value XX - 40
}


// Get vehicle Fuel gauge 0-100%

int getFuel(void) {
  Serial2.print("012F\r");                            // Get from OBD-II the --> Fuel gauge 0-100%
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return strtol(&rxData[6], NULL, 16);                // "41 2F XX" --> Single byte value XX
}


// Get vehicle Battery Voltage in vdc

int getBatt(void) {
  Serial2.print("0142\r");                            // Get from OBD-II the --> Battery Voltage in vdc
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  // Calculation to vdc is based on the following:
  // - "41 42 XX YY" --> Two byte value XX YY
  // - ((high order XX byte *256)+low order YY byte) / 1000
  // - Divide by 1000 to get decimal
  return ((strtol(&rxData[6], NULL, 16) * 256) + strtol(&rxData[9], NULL, 16)) / 1000;
}


// Get car Water Temperature in Celcius

int getCoolant(void) {
  Serial2.print("0105\r");                            // Get from OBD-II the --> Water Temperature in Celcius
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return int(((strtol(&rxData[6], NULL, 16) * 9.0) / 5.0 + 32) - 40);  // "41 05 XX" --> Single byte value XX - 40
}


// Get car odometer reading in Kilometers

int getOdometer(void) {
  Serial2.print("0131\r");                           // Get from OBD-II the --> Odometer reading Kilometers
  waitTime(50);                                      // Wait for reposne to complete before reading
  readOBD();                                         // Read response from board
  // Calculation to Kilometers is based on the following:
  // - "41 0C XX YY" --> Two byte value XX YY
  // - ((high order XX byte *256)+low order YY byte) / 4
  return ((strtol(&rxData[6], NULL, 16) * 256) + strtol(&rxData[9], NULL, 16));
}


// Get car Intake Air Temperature in Celcius

int getIntakeAir(void) {
  Serial2.print("010F\r");                            // Get from OBD-II the --> Intake Air Temperature in Celcius
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return int(((strtol(&rxData[6], NULL, 16) * 9.0) / 5.0 + 32) - 40);  // "41 0F XX" --> Single byte value XX - 40
}


// Get car Intake Air Flow Rate

int getAirFlowRate(void) {
  Serial2.print("0110\r");                            // Get from OBD-II the --> Air Flow Rate
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  // Calculation to vdc is based on the following:
  // - "41 10 XX YY" --> Two byte value XX YY
  // - ((high order XX byte *256)+low order YY byte) / 100
  // - Divide by 100 to get decimal
  return ((strtol(&rxData[6], NULL, 16) * 256) + strtol(&rxData[9], NULL, 16)) / 100;
}


// Get car Ambient Temperature in Celcius

int getAmbientTemp(void) {
  Serial2.print("0146\r");                              // Get from OBD-II the --> Ambient Temperature in Celcius
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return int(((strtol(&rxData[6], NULL, 16) * 9.0) / 5.0 + 32) - 40);  // "41 46 XX" --> Single byte value XX - 40
}


// Get car Trottle position

int getTrottlePos(void) {
  Serial2.print("0111\r");                            // Get from OBD-II the --> Trottle position
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return strtol(&rxData[6], NULL, 16);                // "41 11 XX" --> Single byte value XX
}


// Get car Barometric Preasure

int getBarometric(void) {
  Serial2.print("0133\r");                            // Get from OBD-II the --> Barometric Preasure
  waitTime(50);                                       // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  return strtol(&rxData[6], NULL, 16);                // "41 33 XX" --> Single byte value XX
}


// Initialize interface settings on the OBD board

void InitOBD(void) {
  waitTime(1000);                                     // Wait for a second before resetting the OBD-II
  Serial2.print("ATZ\r");                             // --> Reset OBD
  waitTime(200);                                      // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
  Serial2.print("ATE0\r");                            // --> Echo off
  waitTime(200);                                      // Wait for reposne to complete before reading
  readOBD();                                          // Read response from board
}


// Read an OBD response from the borard and build the character array

void readOBD(void) {
  char c;
  rxIndex = 0;                                        // Reset buffer character index
  markTime2 = millis();                               // Mark start time
  do {
    if (Serial2.available() > 0) {                    // Characters available to process
      c = Serial2.read();                             // Read a character
      if (rxIndex == 31) c = '>';                     // Force termination as we ran out of array space
      if ((c != '>') && (c != '\r') && (c != '\n')) { // Accept only usable charcters
        rxData[rxIndex++] = c;                        // Add the character to the buffer
        Serial.print(c);
      }
    } else {
      if ((millis() - markTime2) > 20000) {           // Timeout after 20 seconds of no OBD response
        Serial.println("OBD-II read timed out!");
        timeoutOBD = true;
      }
    }
  } while ((c != '>') && (timeoutOBD == false));       // The ELM327 ends with ">" as a command prompt for next command
  Serial.println(rxIndex);
  rxData[rxIndex++] = '\0';                           // Terminate the buffer with null; complete a string
}


// To play audio mp3 files representing numbers. Used to call out speed in increments of 5km/h

#ifdef VER_AUDIO
void openPlay(int acnt) {
  caLen = sprintf(caString, "%d.MP3\0", acnt);
  fileName = caString;
  myMP3File = SD.open(fileName);                    // Default read mode
#ifdef VER_MONITORING
  Serial.print("File ["); Serial.print(fileName); Serial.print("] is opened. Len="); Serial.println(caLen);
#endif
  int err = myAudio.writeFrames(myMP3File);
  if (err != 0) {
    Serial.print("File read error ("); Serial.print(err); Serial.println(")");
  }
  myAudio.setVolume(-300);
  myAudio.startPlayer();
  do {
    int err = myAudio.writeFrames(myMP3File);
#ifdef VER_MONITORING
    Serial.print("Error = ");
    Serial.println(err);
#endif
    if (err == 1) {
      Serial.println("End of file!");
      sleep(1);
      myAudio.stopPlayer();
      break;
    }
    usleep(40000);
  } while (err != 1);
}
#endif


// Suplement to delay() fucntion as delay() does not pause all actions and is asynchronous.

void waitTime(int milsec) {
  markTime = millis();
  while (millis() - markTime < milsec) {
  }
}

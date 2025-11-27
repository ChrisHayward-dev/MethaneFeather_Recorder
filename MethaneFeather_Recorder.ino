#include <Wire.h>
#include  <SPI.h>
#include  <RTClib.h>
#include  <SdFat.h>
#include  <RTCZero.h>
#include  <Adafruit_GPS.h>
#include  <I2CScanner.h>

//#define USE_SSD1306
//#define EST_CH4CONC

#ifdef USE_SSD1306
#include  <Adafruit_SSD1306.h>
#define DISP(...) __VA_ARGS__
#else
#define DISP(...) /* __VA_ARGS__ */
#endif

I2CScanner  i2cScanner;

#ifdef USE_TINYUSB
#include  <Adafruit_TinyUSB.h>
#else
#warning    No support for USB disk!!! use TINYUSB!
#endif



/* version report */
const char *versions[] = {
  "Version 0.15",
  " - conditional compilation for SSD1306",
  " - TGS2611 second batch is group 13 R5%(2500ppm) 3.09K",
  "Version 0.14",
  " - Added estimated CH4 concentration",
  "Version 0.13",
  " - Corrections added for A2D input impedance",
  "Version 0.12",
  " - Allowed GPS to try for 30 minutes to get lock",
  " - Increased number of GPS fixes from 30 to 60",
  " - Added code to operate in the event there are no DS18b20s",
  " - Corrected H2 resistivity calculation",
  "Version: 0.11",
  " -corrected GPS time code",
  " -Added code for external H2 sensor",
  " -Added USB disk emulation",
  " -Added ds18b20 node identification",
  " -Improved GPS location"
};
const int16_t nversions = sizeof(versions) / sizeof(char *);

/* Demonstrate the use of convertAndRead().
*/

#define MAX_CONSOLE_WAIT  20000     // pause to allow console connection
#define USBDISK_TIMEOUT   180       // time for USB disk to autoeject
#define BRIDGE_VOLTAGE    1000      // mV for bridge exciter (0-3300)
#define R_REF             20000.0   // reference resistor for system 2 (rev 3) addr 28FF4FD950160383
#define R_SERIES          10000.0   // Series resistance to A2D chip input
#define MAX_GPS_WAIT      1800      // maximum wait time for GPS  
//#define MAX_GPS_WAIT      10      // maximum wait time for GPS  (used for testing w/o waiting for GPS lock)
#define NUM_GPS_FIXES     60        // number of valid GPS fixes required
#define INPUT_IMPEDANCE   2250000.0 // input impedance of the MCP3424 (from the datasheet ... really something more like 1.67M)
#define MAX_SAMPLES_PER_READING 16  // maximum number of samples to average for one reading
//#define INPUT_IMPEDANCE   1430000.0   // input impedance of the MCP3424 as measured w/ open circuit

#define pinOW       5   // One Wire connection
#define pin5Venable 6   // Enable 5 volt boost supply
#define pinHeater2  A1  // Enable heater on TGS2611 (boost supply must also be enabled - note this is also battery monitor!
#define pinHeater1  10  // Enable heater on TGS2611 (board is configured either for heater 1 or heater 2)
#define pinFan1     11  // Enable fan1 (boost supply must also be enabled)
#define pinFan2     12  // Enable fan2 (boost supply must also be enabled)
#define pinRedLED   13  // Standard Feather red LED
#define pinGreenLED  8  // Standard Feather green LED
#define pinExcite   A0  // DAC for bridge exciter.  Vout = value/1023 * 3.3volts
#define pinSDdetect 7   //  SD card detect
#define pinSDselect 4   // SD SPI chip select
#define pinGPSenable A2 // pull low to disable the GPS
#define SSD1306reset A3 // SSD1306 reset pin (if SSD is present)

const uint8_t redLed  = 13;    // Standard Red Led   : Pin26 PA17
const uint8_t greenLed = 8;    // Standard Green Led : Pin11 PA06

const char *header = "Time,Time,extGas,heater,ref,intGas,TGStemp,T2,T3,T4,T5,bmeTemp,bmePres,bmeRH,bmeGas,bme.alt,lat,lon,battery,Rgas,Rgas2";
const char *units  = "sec,datetime,uV,uV,uV,uV,dC,dC,dC,dC,dC,dC,hPa,%,kOhm,m,N,E,volts,ohms,ohms";

uint32_t  timer =  millis();
File dataFile;

SdFat sd;

enum EventType {FAN1_ON, FAN1_OFF, FAN2_ON, FAN2_OFF, DISP_R, PLOT_CH4, PLOT_H2, TAKE_DATA};
typedef struct {
  bool    execd;
  uint8_t tsec;
  enum EventType event ;
} EventList;

#define RECORD_INTERVAL 30;               // recording interval. Must be > max evlist.tsec
EventList evlist[] = {{false, 0, FAN1_ON},
                    {false, 5, FAN2_ON},
                    {false, 6, DISP_R},
                    {false,10, FAN1_OFF},
                    {false,11, PLOT_CH4},
                    {false,15, FAN2_OFF},
                    {false,16, PLOT_H2},
                    {false,25, TAKE_DATA}};
int8_t  nevents = sizeof(evlist)/sizeof(EventList);




void errorHalt(const  char *msg) {
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  Serial.println(msg);
  Serial.println("Error Halt");
  DISP(msg);
  while (true) {
    digitalWrite(redLed, !digitalRead(redLed));
    delay(100);
  }
}


RTCZero rtc;

void topOfMinute() {
  digitalWrite(redLed, LOW);
}
void setup(void)
{
  bool haveBattery = false;
  uint8_t   node = 0;
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(pin5Venable, OUTPUT);
  pinMode(pinHeater1, OUTPUT);
  pinMode(pinFan1, OUTPUT);
  pinMode(pinFan2, OUTPUT);
  pinMode(pinSDdetect, INPUT_PULLUP);
  pinMode(pinGPSenable, OUTPUT);
  digitalWrite(pinGPSenable, HIGH);
  asm(".global _printf_float");
  
  #ifdef USE_TINYUSB
  //DISP("USB Drive Active");
  USBdrive();
  #else
  #warning Skipping USB drive
  #endif

  
  DISP(ssd1306_setup());
 
  DISP("USB Drive Ejt");
  Serial.begin(9600);
  DISP("Wait Console");
  while (!Serial && ((millis() - timer) < MAX_CONSOLE_WAIT)) {
    digitalWrite(redLed, !digitalRead(redLed));
    delay(250);
  }
  delay(500);
  Serial.println("Methane Feather Recorder Demo");
  Serial.print("Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  DISP("Complied on");
  DISP("%s %s",__DATE__,__TIME__);
  for(int k=0;k<nversions;k++) {
    Serial.println(versions[k]);
  }

  delay(5000);  // allow time for SSSD1306 display to be read
  // Initialize the I2C scanner for gps setup and ssd1306 setup
  
  extern int I2Cbus_clear(int,int);
  extern char* I2Cbus_statusstr(int);
  int status = I2Cbus_clear(SDA,SCL);
  if(!status) {
    Serial.println("I2C bus is hung!");
    Serial.println(I2Cbus_statusstr(status));
  }

  Wire.begin();
  i2cScanner.Init();
  i2cScanner.Scan();
  Wire.setClock(400000L);

  // Blink LEDs to indicate we are ready to go
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, HIGH);
  delay(2000);

  // setup the internal clock
  rtc.begin();
  DateTime dt = DateTime(__DATE__, __TIME__);
  rtc.setDate(dt.day(), dt.month(), dt.year());
  rtc.setTime(dt.hour(), dt.minute(), dt.second());

  haveBattery = checkBattery();

  node = dsNodeNumber();
  if (node > 0) {
    Serial.print("Node Number: ");
    Serial.println(node);
  } else {
    Serial.println("Node number needs to be added to software");
    Serial.println(" make note of temperature sensor addresses");
  }
  DISP("Node: %d",node);
 
  
  if (!haveBattery) errorHalt("Cannot detect a battery");
  if (getBattery() < 3.4) errorHalt("Battery Voltage Low");

  if (digitalRead(pinSDdetect) == LOW) errorHalt("Cannot detect SD Card");
  if (!sd.begin(pinSDselect, SD_SCK_MHZ(12))) errorHalt("Card failed to init!");
  char dname[] = "CH4_N00_000.TXT";
  int  fnumber = 1;
  do {
    snprintf(dname, sizeof(dname), "CH4_N%02d_%03d.txt", node, fnumber);

    if (!sd.exists(dname)) break;
    fnumber++;
  } while (fnumber < 999);
  Serial.print("Data filename: ");
  Serial.println(dname);
  dataFile = sd.open(dname, FILE_WRITE);
  if (dataFile) {
    Serial.print("\nData file opened:");
    Serial.println(dname);
    DISP("file: %s",dname);
  } else {
    Serial.print("Unable to open datafile: ");
    Serial.println(dname);
    while (true) {
      digitalWrite(redLed, HIGH);
      delay(250);
      digitalWrite(redLed, LOW);
      delay(200);
    }
  }
  
  delay(2000);    // Pause to allow someone to look at display

  gps_setup();
  ds_setup(&dataFile);        // do the ds_setup first so the dataFile headers come after the therometer addresses
  dataFile.println(header);
  dataFile.println(units);
  dataFile.flush();
  bme_setup();
  mcp_setup();

  exciteSet(BRIDGE_VOLTAGE);  // Enable the bridge exciter at 1 volt
  mcp_setup();

  enable5V(true);
  enableHeater(true);
  Serial.println("Waiting 60 seconds for 5V power to stabilize");
  DISP("5V Pwr 60 sec");
  for(int k=0;k<60;k++) {
    digitalWrite(redLed,!digitalRead(redLed));
    delay(1000);
  }
  mcp_setup();
  Serial.println("5 second test of Fan1");
  digitalWrite(pinFan1, HIGH);
  DISP("Fan 1");
  delay(5000);
  digitalWrite(pinFan1, LOW);
  Serial.println("5 second test of Fan2");
  digitalWrite(pinFan2, HIGH);
  DISP("Fan 2");
  delay(3000);
  digitalWrite(pinFan2, LOW);

  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
  enable5V(false);
  enableHeater(false);

  Serial.println("Searching for GPS");
  //DateTime dc = DateTime(rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  uint32_t startGPS = millis();
  uint32_t notedGPS = millis();
  extern Adafruit_GPS  *GPS;
  int16_t numFixes = 0;
  while ((numFixes<NUM_GPS_FIXES) && (((millis() - startGPS) / 1000) < MAX_GPS_WAIT)) {
    gps_process();
    delay(250);
    digitalWrite(redLed, !digitalRead(redLed));
    if(GPS->fix) numFixes++;
    if ((millis() - notedGPS) > 10000) {
      if(numFixes<1) {
        DISP("GPS: Search %d/%d",(millis() - startGPS)/1000,MAX_GPS_WAIT);
        Serial.print("Searching for GPS for ");
        Serial.print((int16_t)((millis() - startGPS) / 1000));
        Serial.print(" seconds (stop in ");
        Serial.print(MAX_GPS_WAIT - (millis() - startGPS)/1000);
        Serial.println(" seconds)");
      } else {
        GPS->fix = false;   //reset the fix for next time
        DISP("GPS fix %d/%d",numFixes,NUM_GPS_FIXES);
        DISP("");
        Serial.print("Accumulating GPS fixes ");
        Serial.print(numFixes);
        Serial.print(" of ");
        Serial.println(NUM_GPS_FIXES);
        gps_setTime();
      }
      notedGPS = millis();
    }
    delay(980);
  }
  if (numFixes < 3) {
    Serial.println("Did not get a sufficient GPS lock!  Date/Time may be wrong!");
  } else {
    Serial.println("Got at least 3 GPS fixes!");
    DISP("GPS Locked!");
    Serial.print("GPS Time: ");
    gps_printTime(&Serial);
    Serial.println("");
    Serial.print("GPS Location: ");
    gps_printLoc(&Serial);
    Serial.println("");
    Serial.println("GPS is locked now!");
  }
 
  gps_sleep();
  digitalWrite(greenLed, HIGH);
  delay(2000);
  
  DISP("Wait for minute mark");
  // wait for top of minute
  while (rtc.getSeconds() != 0) {
    delay(100);
  }
  // setup the rtc alarm
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(topOfMinute);
  DISP("==MARK===");

  Serial.println("Starting acqusition!");

  
  exciteSet(1000);
  enable5V(true);
  enableHeater(true);
  DISP(ssd1306_printBig("Start"));
}

void loop(void) {

  Stream *out = &dataFile;
  static uint32_t loopCount = 0;
  static bool   takeData = false;
  uint8_t        cursec;
  static uint8_t prevsec=60;
  cursec = rtc.getSeconds() % RECORD_INTERVAL;
  if(cursec < prevsec) {
    for(int k=0;k<nevents;k++) {
      evlist[k].execd = false;
    }
  }
  prevsec = cursec;
  extern float Rgas;
  extern float RgasH;
  extern float logRch4[];
  extern float logRh2[];
  extern float rh;
  extern float temp;
  extern int   logRch4p;
  extern int   logRh2p;
  for (int k=0;k<nevents;k++) {
    if((evlist[k].tsec < cursec) && (!evlist[k].execd)) {
      evlist[k].execd = true;
      // Serial.print("Event Number ");Serial.println(k);
      // Serial.print("Event: ");Serial.println(evlist[k].event);
      switch (evlist[k].event) {
        case FAN1_ON:
          digitalWrite(pinFan1, HIGH);
          break;
        case FAN1_OFF:
          digitalWrite(pinFan1, LOW);
          break;
        case FAN2_ON:
          digitalWrite(pinFan2, HIGH);
          break;
        case FAN2_OFF:
          digitalWrite(pinFan2, LOW);
          break;
        case DISP_R:
          if(Rgas>0) {
            DISP("Bat: %.1f",getBattery());
            mcp_disp();

         
          };
          break;
        case PLOT_CH4:
          if(Rgas>0) {
              DISP(ssd1306_plot(Rgas,logRch4,logRch4p));
              DISP(ssd1306_labelPlot("CH4"));
          };
          break;
        case PLOT_H2:
          if(RgasH>0) {
            DISP(ssd1306_plot(RgasH,logRh2,logRh2p));
            DISP(ssd1306_labelPlot("H2"));
          };
          break;
        case TAKE_DATA:
          gps_process();
          DISP(""); // clean up display
          DISP("");
          takeData = true;
          break;
        default:
          Serial.print("Unknown event: ");Serial.println(evlist[k].event);
      }
    }
  }
  loopCount++;
  //Serial.print("Loop: ");Serial.println(loopCount);
  mcp_process();
  yield();
  
  if (takeData) {
    takeData = false;
    digitalWrite(greenLed, !digitalRead(greenLed));
    gps_printTime(out);
    ds_process();   // start the DS18B20 temperature reading to interleave w/ mcp
    bme_process();  // start the BME reading to interleave as well
    mcp_process();
    mcp_printVals(out);
    ds_printTemps(out);
    bme_printReadings(out);
    gps_printLoc(out);
    soh_printBattery(out);
    mcp_printGas(out);

    int SensorID = 12;
    float c=concen(SensorID,Rgas/1000.0,rh,temp,19.0);
    char bufr[30];
    snprintf(bufr,sizeof(bufr),"%.1f",c);
    DISP(ssd1306_printBig(bufr));
    
    out->println(-999);
    out->flush();

    
    mcp_printGas(&Serial);
    Serial.println("");
  }

  //  if (rtc.getSeconds() > 32) {
  //    digitalWrite(greenLed,LOW);
  //    digitalWrite(redLed,HIGH);
  //    delay(100);
  //    rtc.standbyMode();
  //  }

  // if VBAT < 3.4 volts, then we need to shut down everyting on the VBAT line since
  //  the protrection circuitry on the Feather boards won't shut down the power discharge
  //  on the VBAT line itself

  if (getBattery() < 3.35) {
    rtc.disableAlarm();
    Serial.println("Battery is low!  Shutdown!");
    DISP("Shutdown: Low Battery!");
    delay(2000);
    set_sleepPower();
    digitalWrite(greenLed, LOW);
    digitalWrite(redLed, HIGH);
    while (true) {
      __WFI();
    }
  }
  delay(100);
}

// used just before sleep.  Turn off all the externals
void set_sleepPower() {
  digitalWrite(pinGPSenable, HIGH);
  digitalWrite(pinFan1, LOW);
  digitalWrite(pinFan2, LOW);
  enableHeater(false);
  enable5V(false);
  exciteSet(0);
}
void set_wakePower() {
  //digitalWrite(pinGPSenable, LOW);
  digitalWrite(pinFan1, HIGH);
  digitalWrite(pinFan2, HIGH);
  enable5V(true);
  enableHeater(true);
  exciteSet(BRIDGE_VOLTAGE);
}

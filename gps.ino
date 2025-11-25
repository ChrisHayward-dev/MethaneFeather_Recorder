// GPS routines
#include  <Adafruit_GPS.h>

#define   GPSSerial Serial1
#define   GPSECHO   false

Adafruit_GPS *GPS;

bool  GPS_is_I2C = false;
void gps_printTime(Stream *device) {
  DateTime rtctime = DateTime(rtc.getYear(),rtc.getMonth(),rtc.getDay(),rtc.getHours(),rtc.getMinutes(),rtc.getSeconds());
  device->print(rtctime.unixtime() /*+1024L*7*86400*/);
  device->print(",");
  device->print("\"20");
  device->print(rtc.getYear());
  device->print("/");
  device->print(rtc.getMonth());
  device->print("/");
  device->print(rtc.getDay());
  device->print("T");
  device->print(rtc.getHours());
  device->print(":");
  device->print(rtc.getMinutes());
  device->print(":");
  device->print(rtc.getSeconds());
  device->print('"');
  device->print(",");
  DISP("GPS Time");
  DISP("20%02d/%02d/%02d %02d:%02d:%02d",rtc.getYear(),rtc.getMonth(),rtc.getDay(),rtc.getHours(),rtc.getMinutes(),rtc.getSeconds());
}

void gps_sleep() {
  digitalWrite(A2, HIGH); //disable GPS if it is enabled view A2
  digitalWrite(redLed, LOW);
  // GPS->standby();      // FIXME: GPS into standby makes I2C go away
}
void gps_setTime() {
  Serial.print("GPS time:");
  gps_printTime(&Serial);
  // Serial.print(GPS->year);Serial.print("-");
  // Serial.print(GPS->month);Serial.print("-");
  // Serial.println(GPS->day);
  rtc.setDate(GPS->day,GPS->month,GPS->year);
  rtc.setTime(GPS->hour,GPS->minute,GPS->seconds);
  
}
void gps_printLoc(Stream *device) {
  device->print(GPS->latitude, 5);
  device->print(",");
  device->print(GPS->longitude, 5);
  device->print(",");  
}


void gps_setup() {
  Serial.println("\nGPS setup");
  Serial.println("Checking for I2C GPS");
 
  if(i2cScanner.Check(GPS_DEFAULT_I2C_ADDR)) {
    GPS_is_I2C = true;
    GPS = new Adafruit_GPS(&Wire);
    Wire.begin();
    if(GPS->begin(0x10)) {
      Serial.println("Attached a GPS on the I2C bus");
    } else {
      Serial.println("Failed to attach GPS to I2C bus");
    }
  } else {
    GPS_is_I2C = false;
    GPS = new Adafruit_GPS(&GPSSerial);
    GPS->begin(9600);
    Serial.println("Attempting to attach a GPS on Serial1");
  }

  // Not all GPS will have power control on A2 -- but we'll do it anyway
  pinMode(A2,OUTPUT);
  digitalWrite(A2,LOW);  //enable GPS power
  delay(1000);
  Serial.println("Sending wakeup commands to the GPS");
  delay(10);
  GPS->wakeup();
  Serial.println("Wakeup sent");
  delay(100);
  #define PMTK_POWERUP  "$PMTK225,0*2B"
  GPS->sendCommand(PMTK_POWERUP);
  GPS->sendCommand(PMTK_AWAKE);
  GPS->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  Serial.println("Initialized the GPS");
  delay(1000);
}

bool gps_process() {
  while (char c = GPS->read()) {
    if (GPSECHO) Serial.print(c);
    if (GPS->newNMEAreceived()) {
      if (!GPS->parse(GPS->lastNMEA())) {
        continue;
      }
    }
  }
  if (GPS->secondsSinceFix() <= 0.5) {
    float s = GPS->seconds + GPS->milliseconds / 1000. + GPS->secondsSinceTime();
    int m = GPS->minute;
    int h = GPS->hour;
    int d = GPS->day;
    // Adjust time and day forward to account for elapsed time.
    // This will break at month boundaries!!! Humans will have to cope with
    // April 31,32 etc.
    while (s > 60) {
      s -= 60;
      m++;
    }
    while (m > 60) {
      m -= 60;
      h++;
    }
    while (h > 24) {
      h -= 24;
      d++;
    }
    // Kluge: Adjust the DS3231 each time we are close to on time.
    // We keep this in case we need to run a DS3231 as well as GPS
    //rtc.adjust(DateTime(GPS->year + 2000, GPS->month, GPS->day, h, m, (int)s));
  }
  return(true);
}

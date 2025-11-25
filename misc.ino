/* Misc functions here
 * enableHeater - enable heater 2
   enable5V - enable the 5V supply
   exciteSet - set exciter voltage
   doScan - check for ds18b20 temperature sensors
   scanPins - check the status of various control pins
   checkFans - exercise the fans
   checkBattery - report connected battery voltage
   I2Cscanner - report devices on I2C bus
*/
// enable Heater 1 (heater 2 is reserved for second board)
bool enableHeater(bool flag) {
  pinMode(pinHeater1,OUTPUT);
  if(flag) {
    digitalWrite(pinHeater1,HIGH);
  } else {
    digitalWrite(pinHeater1,LOW);
  }
  return(true); 
}
// enable 5volt supply
bool enable5V(bool flag) {
  pinMode(pin5Venable,OUTPUT);
  if(flag) {
    digitalWrite(pin5Venable,HIGH);
  } else {
    digitalWrite(pin5Venable,LOW);
  }
  return(true);
}
// set exciter voltage
// Vout = value/1023 * 3.3volts
bool exciteSet(int mV) {
  int aValue = 0;
  if (mV < 0 || mV > 3300) {
    Serial.println("Invalid exciter request - must be beween 0 and 3300");
    mV = 0;
    return(false);
  }
//  Serial.print("Requesting exciter voltage of ");
//  Serial.print(mV);
//  Serial.println(" mV");
  aValue = (1023L * (long)mV) / 3300L;
//  float vout = aValue * 3300.0 / 1023.0;
  analogWrite(A0,aValue);
//  Serial.print("Exciter set to ");
//  Serial.print(aValue);
//  Serial.print("(");
//  Serial.print(vout,1);
//  Serial.println("mV)");
  return(true);
}
bool soh_printBattery(Stream *device) {
  float measuredvbat = analogRead(A7);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  device->print(measuredvbat,2);
  device->print(",");

   return (false);
}
float getBattery() {
  float measuredvbat = analogRead(A7);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  return(measuredvbat);
}

bool scanPins() {
  // if everything was connected correctly, all pins should be low
  // even if pullup is enabled except one of the two heater enables

  // set A0 to 0 for the initial work to set the bridge supply to 0 volts
  Serial.println("Set bridge exciter to 0 volts");
  analogWrite(A0, 0);

  pinMode(pin5Venable, INPUT_PULLUP);
  pinMode(pinHeater2, INPUT_PULLUP);
  pinMode(pinFan1, INPUT_PULLUP);
  pinMode(pinFan2, INPUT_PULLUP);

  checkPin(pin5Venable, "5V enable");
  checkPin(pinHeater2, "Heater 2 enable");
  checkPin(pinFan1, "Fan 1 enable");
  checkPin(pinFan2, "Fan 2 enable");
    /* Pin Heater 1 is also battery monitor on pin A7 - check this on A7 to decide if pin is an enable
     */
  Serial.print("Pin Heater 1 reports ");
  Serial.print(analogRead(A7));
  Serial.println(" counts");
  if(analogRead(A7)>50) {
    Serial.println("Pin Heater 1 probably not connected");
  } else {
    pinMode(pinHeater1, INPUT);
    checkPin(pinHeater1, "Heater 1 enable");
  }
  // Pin Heater 1 is also on the battery monitor pin.  We mostly don't want to use this one
  return (true);
}
bool checkFans() {
  pinMode(pinFan1, OUTPUT);
  pinMode(pinFan2, OUTPUT);
  digitalWrite(pinFan2, LOW);
  digitalWrite(pinFan1, LOW);
  Serial.println("Checking FAN1...");
  digitalWrite(pinFan1, HIGH);
  delay(10000);
  digitalWrite(pinFan1, LOW);
  delay(2000);
  Serial.println("Checking Fan2...");
  digitalWrite(pinFan2, HIGH);
  delay(10000);
  digitalWrite(pinFan2, LOW);
  Serial.println("Fan check complete -- did they run?");
  return (true);
}

bool checkBattery() {
  pinMode(pinHeater1, INPUT);
  float measuredvbat = analogRead(A7);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  Serial.print("VBat: ");
  Serial.print(measuredvbat);
  Serial.println(" Volts");
  if (measuredvbat > 3.3) {
    Serial.println("Battery detected");
    return (true);
  }
  Serial.println("No battery detected");
  return (false);
}
bool checkPin(int pinNum, const char *pinName)  {
  Serial.print(pinName);
  Serial.print("(");
  Serial.print(pinNum);
  Serial.print(")");
  Serial.print(" is ");
  pinMode(pinNum, INPUT);
  delay(100);
  if (digitalRead(pinNum) == HIGH) {
    Serial.print("HIGH and ");
  } else {
    Serial.print("LOW and ");
  }
  if (digitalRead(pinNum) == HIGH) {
    Serial.println(" is NOT CONNECTED!!");
    return (false);
  } else {
    Serial.println(" appears to be connected");
  }
  return (true);
}


// I2C scanner
int i2cScan() {
  byte error, address;
  int nDevices;
  Serial.println(">>I2C Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("    I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print("  - ");
      switch (address) {
        case 0x6F:
          Serial.print("MCP3424 - address jumper open");
          break;
        case 0x77:
          Serial.print("BME688");
          break;
        default:
          Serial.print("Unknown");
      }
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("    done");
  return (nDevices);
}

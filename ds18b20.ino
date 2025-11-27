// ds18b20 routines
#include <OneWire.h>
#include <DallasTemperature.h>

#define numThermometers 5
OneWire             oneWire(pinOW);
DallasTemperature   tempSensors(&oneWire);
DeviceAddress       thermometer[numThermometers];

void ds_process() {
  tempSensors.setWaitForConversion(false);
  tempSensors.requestTemperatures();
}

void ds_printTemps(Stream *device) {
    if(tempSensors.getDeviceCount()>0)  {
    while(tempSensors.isConversionComplete() == false) {
      delay(10);
    }
    for(int k=0;k<tempSensors.getDeviceCount();k++) {
      device->print(tempSensors.getTempC(thermometer[k]),3);
      device->print(",");
    }
  }
  for(int k=tempSensors.getDeviceCount();k<numThermometers;k++) {
    device->print(","); //pad out for missing ds18b20s
  }
};

DeviceAddress nodes[]={{0x28,0x78,0x41,0xE5,0x0F,0x00,0x00,0x94},
                       {0x28,0xFD,0x62,0xE6,0x0F,0x00,0x00,0xAB},
                       {0x28,0x9D,0x33,0xCC,0xA1,0x22,0x01,0x57},
                       {0x28,0xAF,0xF7,0xE5,0x0F,0x00,0x00,0x06},
                       {0x28,0x5C,0x57,0xE6,0x0F,0x00,0x00,0x96},
                       {0x28,0xB5,0x58,0xE5,0x0F,0x00,0x00,0xCE},
                       {0x28,0x47,0x56,0xF3,0xA1,0x22,0x01,0x4D},
                       {0x28,0xE8,0xAD,0xE5,0x0F,0x00,0x00,0xDE},
                       {0x28,0x5C,0x73,0xE6,0x0F,0x00,0x00,0x71},
                       {0x28,0xC9,0x82,0xE6,0x0F,0x00,0x00,0x40},
                       {0x28,0x3B,0x79,0xE5,0x0F,0x00,0x00,0x02},
                       {0x28,0x47,0x7A,0xF2,0xA1,0x22,0x01,0x1B},
                       {0x28,0x1D,0x18,0xE6,0x0F,0x00,0x00,0x52},
                       {0x28,0x0D,0x38,0xE6,0x0F,0x00,0x00,0xF1},
                       {0x28,0xEF,0x97,0xE5,0x0F,0x00,0x00,0x62},
                       {0x28,0x82,0x9C,0xE5,0x0F,0x00,0x00,0x9B},
                       {0x28,0x44,0x80,0xFE,0xA1,0x22,0x01,0x4B},
                       {0x28,0xBF,0x3E,0xE6,0x0F,0x00,0x00,0x04},
                       {0x28,0xFF,0x4F,0xD9,0x50,0x16,0x03,0x83},
                       {0x28,0x6C,0x0F,0xF2,0xA1,0x22,0x01,0xEA},
                       {0,0,0,0,0,0,0,0}};
const int8_t numNodes = sizeof(nodes)/sizeof(DeviceAddress);
uint8_t nodeNumbers[numNodes+1] = {7,16,19,8,15,2,6,18,3,4,7,20,11,9,12,14,5,1,17,30,0};
uint8_t dsNodeNumber(){
  uint8_t nodeNum = 0;
  tempSensors.begin();
  for (int k = 0; k < tempSensors.getDeviceCount(); k++) {
    if (!tempSensors.getAddress(thermometer[k], k)) {
      Serial.print("Unable to get thermometer ");
      Serial.print(k);
      Serial.println(" address");
    } else {
      //Serial.print("Checking ");Serial.print(numNodes);Serial.println(" nodes");
      for(int k2=0;k2<numNodes;k2++){
        //Serial.print("Considering item ");
        //Serial.println(k2);
        nodeNum = 0;
        for(int k3=0;k3<8;k3++) {
          //Serial.print(nodes[k2][k3],HEX);
          //Serial.print(",");
          if(nodes[k2][k3] != thermometer[k][k3]) {
            nodeNum = 0;
            break;
          } else {
            nodeNum = nodeNumbers[k2];
          }
          //Serial.println();
        }
        if(nodeNum > 0) {
          return(nodeNum);
        }
      }
    }
    Serial.print("Unknown Thermometer ");Serial.print(k);Serial.println(":");
    Serial.print("{");
    for(int k3=0;k3<8;k3++) {
      Serial.print("0x");
      if(thermometer[k][k3]<16) Serial.print("0");
      Serial.print(thermometer[k][k3],HEX);
      if(k3<7) Serial.print(",");
    }
    Serial.println("}");
    
  }
  return(0);
}
bool dsPrintAddr( uint8_t *addr, Stream *dev ) {
  dev->print("{");
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    dev->print("0x");
    if (addr[i] < 16) Serial.print("0");
    dev->print(addr[i], HEX);
    if(i<7)dev->print(",");
  }
  dev->println("}");
  return(true);
}


bool ds_setup(Stream *dev) {
  Serial.print("Locating DS18b20 temperature sensors... found ");
  tempSensors.begin();
  Serial.print(tempSensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if(tempSensors.getDeviceCount() > 0) {
    Serial.print("Parasite power is: ");
    if (tempSensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
    for (int k = 0; k < tempSensors.getDeviceCount(); k++) {
      if (!tempSensors.getAddress(thermometer[k], k)) {
        Serial.print("Unable to get thermometer ");
        Serial.print(k);
        Serial.println(" address");
      }
      Serial.print("Device ");
      Serial.print(k);
      Serial.print(" address: ");
      dsPrintAddr(thermometer[k],&Serial);
      dev->print("Temp Sensor ");
      dev->print(k);
      dev->print(" address: ");
      dsPrintAddr(thermometer[k],dev);
      tempSensors.setResolution(thermometer[k], 12);
      Serial.print("  resolution:");
      Serial.println(tempSensors.getResolution(thermometer[k]), DEC);
    }
    Serial.print("Device 0 Resolution: ");
    Serial.println(tempSensors.getResolution(thermometer[0]), DEC);
    tempSensors.requestTemperatures();
    tempSensors.setWaitForConversion(false);
    Serial.println("Waiting for temperature conversion");
    uint32_t stime = millis();
    while ((tempSensors.isConversionComplete() == false)&& ((millis()-stime)<10000UL)) {
      Serial.print(".");
      delay(100);
    }
    Serial.println();
    for (int k = 0; k < tempSensors.getDeviceCount(); k++) {
      Serial.print("Device ");
      Serial.print(k);
      Serial.print(" ");
      Serial.print(tempSensors.getTempC(thermometer[k]), 3);
      Serial.print(" deg C - resolution:");
      Serial.println(tempSensors.getResolution(thermometer[k]));
    }
  }
  return (true);
}

// BME688
//  We use the Adafruit library to simplify things
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

Adafruit_BME680 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
float rh = 30.0;
float temp = 21;

void bme_process() {
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme.beginReading();
}
void bme_printReadings(Stream *device) {
  
  if(!bme.endReading()) {
    device->print(",,,,,");
    return;
  }
  device->print(bme.temperature,3);
  temp = bme.temperature;
  device->print(",");
  device->print(bme.pressure / 100.0);
  device->print(",");
  device->print(bme.humidity);
  rh = bme.humidity;
  device->print(",");
  device->print(bme.gas_resistance / 1000.0);
  device->print(",");
  device->print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  device->print(",");
}
bool bme_setup() {
  Serial.println("BME688 check");
  if (!bme.begin()) {
    Serial.println("Unable to find a BME680 or BME688 sensor");
    return (false);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 1000); // 320*C for 150 ms (chaned to 1 second - cth)

  // make a set of readings
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return (false);
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  return (true);
}

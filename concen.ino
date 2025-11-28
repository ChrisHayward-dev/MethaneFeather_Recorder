// Methane Feather Concentration calculation
/* This is based upon concentration calculations made in the matlab routine TGS2611Calculations.mlx
    It assumes that the total resistivity is the rpoduct of a fixed resistance in parallel at 0 ppm concentrations and a resistance that is the result of methan concentration

*/
#include  <math.h>
  // note resistivity is in Kohms
  // For group 12 sensors, datasheet values are 2.80 3.05 1.69 1.47 corresponding to LEL 5%, 10%, 15%, 20%
  // Rp should be 21.7874, M= 1.8612, R@3ppm = 18.5
  // for group 13 sensors, datasheet values are 3.09 2.26 1.87 1.62
  // Rp should be 21.9890, M= 1.9331, R@3ppm = 26.4
  
  float Rp = 21.7874; // calculated R parallel
  float Cr = 2500;    // reference concentration (5%)
  float Rr = 3.3054;  // reference resistivity (corrected from table using Rp)
  float M  = 1.8612;  // slope of log10 Cr/Rr curve
  float Mrh= -0.2735; // slope of humidity correction
  float K  = 0.6696;  // intersection of humidition correction
  float C0 = 3.0; // CH4 ppm in ambient conditions

void concen_printGas(Stream *device,float c) {
  device->print(c);
  device->print(",");
}

float CH4concen(int SensorID,float Rx,float rh,float T,float R0) {
  switch (SensorID) {
    case 12:
    Rp = 22.9694;
    M  = 1.9707;
    break;
    case 13:
    Rp = 37.090;
    M  = 2.0286;
    break;
    default:
      Serial.println("Unknown sensorID.  Must be 12 or 13");
      return(0.);
  }
  float Ps = 610.04 * exp(17.625*T/(T+243.04));
  float Rw = 461.5;                                 // Specific gas constant for water valop
  float Tk = 273 + T;                               // Temperature in Kelvin
  float ah = rh * Ps / (Rw * Tk * 100.0) * 1000.0;  // absolute humidity
  float cf = pow(ah, Mrh) * exp(K);                 // humidity correction factor

  float R0g = Rr * pow(C0 / Cr,-1.0/M);   // calculated R at ambient
  // R0 = R0 / cf;                           // correct R0 to ambient
  float Rp = 1.0 / (1.0/R0 - 1.0/R0g);    // calculated Rp

  float Rxc = Rx;
  // Rxc =  Rx / cf;                              // corrected Rx for humidity
  float Rg  = 1.0 / (1.0/Rxc - 1.0/Rp);
  float Cx  = Cr * pow( Rr / Rg, M);

#ifdef EST_CH4CONC
  Serial.print("Est CH4: ");Serial.print(Cx,1);Serial.println(" ppm");
#endif
  return(Cx);
}

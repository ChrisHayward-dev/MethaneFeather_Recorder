// Methane Feather Concentration calculation
/* This is based upon concentration calculations made in the matlab routine TGS2611Calculations.mlx
    It assumes that the total resistivity is the rpoduct of a fixed resistance in parallel at 0 ppm concentrations and a resistance that is the result of methan concentration

*/
#include  <math.h>
  // note resistivity is in Kohms
  float Rp = 21.7874; // calculated R parallel
  float Cr = 2500;    // reference concentration
  float Rr = 3.3054;  // reference resistivity
  float M  = 1.8612;  // slope of log10 Cr/Rr curve
  float Mrh= -0.2735; // slope of humidity correction
  float K  = 0.6696;  // intersection of humidition correction
  float C0 = 3.0; // CH4 ppm in ambient conditions


float concen(float Rx,float rh,float T,float R0) {
   
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

  Serial.print("Est CH4: ");Serial.print(Cx,1);Serial.println(" ppm");
  return(Cx);
}

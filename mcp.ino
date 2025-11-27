// MCP check routines
#include <MCP342x.h>

uint8_t mcpAddress = 0x6f;  // 6e is the address w/ jumper open
MCP342x adc = MCP342x(mcpAddress);



float mcpLSB = (2.0 * 2.048) / pow(2.0, 18);

long mcp_value[5]; //we use 5 here so we can number by channel starting at 1
long mcp_avg[5];
int  mcp_nsamples = 0;

// We are going to assume that the INPUT_IMPEDCANCE only applies when the
//  measurement is actually made.  It may indeed be almost ignored for those
//  inputs that have a large cap on them since the RC constant coule be much
//  longer than the A2D conversion time
float mcp_calcR(float Vdiff,float Vref,float *Vsup,float Rr = R_REF) {

#ifdef INPUT_IMPEDANCE
  float Rt      = Rr + 1 / (1/Rr + 1/INPUT_IMPEDANCE);
#else
  float Rt      = 2*Rr;
#endif

  float Vsupply = Vref * Rt / (Rt-Rr);
  float Vgas    = Vdiff + Vref;
  float Rn      = Vgas * (Rr/(Vsupply - Vgas));
  *Vsup = Vsupply;
  // Serial.print("  Vdiff: ");Serial.println(Vdiff,3);
  // Serial.print("   VRef: ");Serial.println(Vref,3);
  // Serial.print("Vsupply: ");Serial.println(Vsupply,3);
  // Serial.print("   VGas: ");Serial.println(Vgas,3);
  // Serial.print("     Rn: ");Serial.println(Rn/1000.0,3);
 
  return(Rn);
}

float Rgas = -1;   //make these global to simplify the charting
float RgasH = -1;
void mcp_printGas(Stream *device) {
  float Vsupply;
  float Vref     = mcpLSB *   mcp_value[3];                   // voltage on reference half bridge
  float Vdiff    = mcpLSB *   mcp_value[4] * 1.015;           // this is an empirical correction based upon 1 board and 2 R values
  float VdiffH   = mcpLSB *   mcp_value[1] * 1.015;

  // Serial.println("");
  // for (int k=0;k<5;k++) {
  //   float Rr = (k-2)*500 + R_REF;
  //   Serial.print("Rref: ");
  //   Serial.println(Rr);
  //   Serial.println("CH4");
  //   Rgas = mcp_calcR(Vdiff,Vref,&Vsupply,Rr);
  //   Serial.println("H2");
  //   RgasH= mcp_calcR(VdiffH,Vref,&Vsupply,Rr);   
  // }
  //Serial.println("CH4");
  Rgas = mcp_calcR(Vdiff,Vref,&Vsupply);
  //Serial.println("H2");
  RgasH= mcp_calcR(VdiffH,Vref,&Vsupply);           
  //Serial.println("=============================");

  device->print(Rgas);
  device->print(",");
  device->print(RgasH);
  device->print(",");
  
  


//  Serial.print("Vsupply:");Serial.print(Vsupply,5);
//  Serial.print(", Vref:");Serial.print(Vref.5);
//  Serial.print(", Vdiff:");Serial.print(Vdiff,5);
//  Serial.print(", Vgas:");Serial.print(Vgas,5);
//  Serial.print(", Rgas:");Serial.println(Rgas);
//  Serial.print(Rgas);
//  Serial.print(" ");
//  Serial.println(RgasH);
}

void mcp_disp() {
  static float prevRgas = 0;
  static float prevRgasH= 0;
  DISP("CH4: %.3fK",Rgas/1000.0);
  DISP(" H2: %.3fK",RgasH/1000.0);
  DISP("Del: %.3fK %.3fK",(Rgas - prevRgas)/1000.0,(RgasH - prevRgasH)/1000.0);
  prevRgas = Rgas;
  prevRgasH= RgasH;
}
void mcp_printVals(Stream *device) {
  for(int k=1;k<5;k++) {
    mcp_value[k] = mcp_avg[k]/mcp_nsamples;
    mcp_avg[k] = 0;
    if(k==2) {
      device->print(mcp_value[k]*mcpLSB*3.378,6);
    } else {
      device->print(mcp_value[k]*mcpLSB,6);
    }
    device->print(",");
  }
  mcp_nsamples = 0;
}
void mcp_process() {
  if(mcp_nsamples < MAX_SAMPLES_PER_READING) {
    mcp_readChan(1);
    mcp_readChan(2);
    mcp_readChan(3);
    mcp_readChan(4);  
  
    mcp_nsamples++;
    for(int k=1;k<5;k++) {
      mcp_avg[k] += mcp_value[k];
    }
  }
}

bool  mcp_readChan(int chan) {
  MCP342x::Config status;
  int err = 27;
  while(err != 0) {
    switch (chan) {
      case 1:
        err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
                                MCP342x::resolution18, MCP342x::gain1,
                                1000000, mcp_value[1], status);
        break;
      case 2:
        err = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
                                MCP342x::resolution18, MCP342x::gain1,
                                1000000, mcp_value[2], status);
        break;
      case 3:
        err = adc.convertAndRead(MCP342x::channel3, MCP342x::oneShot,
                                MCP342x::resolution18, MCP342x::gain1,
                                1000000, mcp_value[3], status);
        break;
      case 4:
        err = adc.convertAndRead(MCP342x::channel4, MCP342x::oneShot,
                                MCP342x::resolution18, MCP342x::gain1,
                                1000000, mcp_value[4], status);
        break;
      default:
        Serial.println("Unknown Channel!");
        return (false);
    }
  }
  if (err) {
    Serial.print(">>>Convert Error:");
    Serial.println(err);
    return(false);
  }
  return(true);
}

void mcp_setup() {
    Wire.begin();
  MCP342x::Config status;
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  // Check device present
  Wire.requestFrom(mcpAddress, (uint8_t)1);
  while (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(mcpAddress, HEX);
    delay(2000);
  }
  Serial.println("Identified MCP3424"); //FIXME: Need to do whoami for ident.
  Serial.print("Using LSB of ");
  Serial.print(mcpLSB, 6);
  Serial.println(" volts/bit");

  // check MCP A2D channels in order
  mcpReadChannel(1,"External TGS2611");
  mcpReadChannel(2,"5 volt monitor");
  mcpReadChannel(3,"Bridge Reference");
  mcpReadChannel(4,"Internal TGS2611");

  Serial.print("Heater Voltage: ");Serial.print(mcpLSB * mcp_value[2] * 3.378); Serial.println(" volts");
  return;
}

bool mcpReadChannel(uint8_t chanNum, const char *chanName) {
  MCP342x::Config status;
  mcp_readChan(chanNum);

  Serial.print("MCP3424 channel ");
  Serial.print(chanNum);
  Serial.print(" - ");
  Serial.print(chanName);
  Serial.print(":");
  Serial.print(mcp_value[chanNum]);
  Serial.print(" (");
  Serial.print(mcp_value[chanNum] * mcpLSB, 6);
  Serial.println(" volts)");
  return(true);
}

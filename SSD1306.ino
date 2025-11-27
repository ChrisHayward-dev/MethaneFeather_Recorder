// SSD1306 routines for methane feather recorder
#ifdef USE_SSD1306
#include  <Wire.h>
#include  <Adafruit_GFX.h>
#include  <Adafruit_SSD1306.h>
#include  <OLED_SSD1306_Chart.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define BUFRLEN       22 // Maximum text length for SSD1306
#define MAXPOINTS     20 // Maximum number of points to plot
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
OLED_SSD1306_Chart display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool  haveSSD1306 = false;

                                        // we add an additional 1 point as overflow detection
float logRch4[MAXPOINTS+1] = {0.0};    // buffer of points to plot
float logRh2[MAXPOINTS+1] = {0.0};
int logRch4p = 0;            // current point in circular buffer
int logRh2p  = 0;
float Rmin = 0;
float Rmax = 0;           // min max for plotting limits

void ssd1306_labelPlot(char *c) {
  display.setCursor(50,16);
  display.print(c);
  display.display();
}
void ssd1306_plot(float R,float logV[],int &curpt) {
  char Rlower[10];
  char Rupper[10];
  if(haveSSD1306) {
    logV[curpt] = log10(R);
    Rmin = 10;
    Rmax = 0;
    if(logV[1]==0) {
      for(int k=1;k<MAXPOINTS;k++) {
        logV[k] = 0.9 * logV[curpt];
      }
    }
    for (int k=0;k<MAXPOINTS;k++) {
      Rmin = min(logV[k],Rmin);
      Rmax = max(logV[k],Rmax);
    }
    snprintf(Rlower,sizeof(Rlower),"%.1f",Rmin);
    snprintf(Rupper,sizeof(Rupper),"%.1f",Rmax);
    // Serial.print("Rmin: ");Serial.print(Rmin);Serial.print(" ");Serial.println(Rlower);
    // Serial.print("Rmax: ");Serial.print(Rmax);Serial.print(" ");Serial.println(Rupper);

  
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);    //restart the display
    display.clearDisplay();
    display.setChartCoordinates(0,30);
    display.setChartWidthAndHeight(123,28);
    display.setXIncrement(123/MAXPOINTS);
    display.setYLimits(Rmin,Rmax);
    display.setYLimitLabels(Rlower,Rupper);
    display.setYLabelsVisible(true);
    display.setAxisDivisionsInc(12,6);
    display.setPlotMode(SINGLE_PLOT_MODE);
    char actualThickness = NORMAL_LINE;
    display.setLineThickness(actualThickness);
    display.drawChart();
    display.display();
    curpt++;
    curpt = curpt % MAXPOINTS;
    int indx = curpt;
    for(int k=0;k<MAXPOINTS;k++) {
      display.updateChart(logV[indx]);
      // Serial.print(logV[indx]);
      // Serial.print(" ");
      indx++;
      indx = indx % MAXPOINTS;
    }
    // Serial.println();
  }
  return;
}
 
bool ssd1306_setup() {
  Wire.begin();
    if(i2cScanner.Check(SCREEN_ADDRESS)) {
      haveSSD1306 = true;
      //Serial.println("Have SSD1306 Display");
    } else {
      haveSSD1306 = false;
      //Serial.println("No SSD1306 Display");
      return(false);
    }
  Wire.begin();
  Wire.setClock(400000L);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    haveSSD1306 = false;
    return(false);
  }
  //Serial.println("Attached SSD1306");
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  ssd1306_printBig("SMU");
  display.display();
  delay(5000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("CH2/H2 recorder");
  display.display();
  return(true);
}

void ssd1306_printBig(const char *msg) {
  if(haveSSD1306) {
    display.clearDisplay();
    display.setTextSize(4);
    display.setCursor(0,0);
    display.println(msg);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
  }
}

void ssd1306_printf(const char *format,...) {
  va_list aptr;
  static char bufr[4][BUFRLEN];
  static int ptr=0;
  if(!haveSSD1306) return;

  va_start(aptr, format);
  vsnprintf(bufr[ptr],BUFRLEN-1,format,aptr);
  va_end(aptr);
  ptr++;
  ptr = ptr % 4;
  display.clearDisplay();
  display.setCursor(0,0);
  for(int k=0;k<4;k++) {
    display.println(bufr[(ptr+k)%4]);
  }
  display.display();
}
#endif

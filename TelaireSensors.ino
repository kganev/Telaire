//#define CPMonitorOP  //enable or disable monitoring on serial output, alternate with display, 115200 baud
#define OELCD_OP    //enable or disable LCD Output, alternate with serial output

#include "Wire.h"
#include <SPI.h>
#include <PinChangeInterrupt.h>

//  Width Guide      "---------------------"
#define SplashScreen "Telaire all, v1.2"
char ScreenHeader[] = "  Amphenol Sensors";
char ScreenFooter[] = " Telaire Technology";
char DisplayUnit[] = "ppm";

#ifdef OELCD_OP
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "OELCD.h"
#endif
#include "T9602.h"
#include "T6713.h"
#include "SM-PWM-01C.h"

void setup()
{
#ifdef CPMonitorOP //output to PC through USB
  Serial.begin(115200);  // start serial for output
  Serial.println(SplashScreen);
  Serial.println("CO2ppmValue, temperature, humidity, PM 2.5, PM 10, AQIColour");
#endif

  Wire.begin();
#ifdef OELCD_OP
  displaySetupScreen();
#endif

  SWM_PM_SETUP();
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  attachPCINT(digitalPinToPCINT(PM10_IN_PIN), calcPM10, CHANGE);
  attachPCINT(digitalPinToPCINT(PM2_IN_PIN), calcPM2, CHANGE);
}


void loop() {
  if (millis() >= (samplerate + sampletime))
  {
    getT6713data();
    getT9602data();
    CalculateDustValue();

#ifdef OELCD_OP
    displayReading(CO2ppmValue, temperature, humidity, ScreenHeader, ScreenFooter, AQIColour);
#endif

#ifdef CPMonitorOP //output to PC through USB
    Serial.print(CO2ppmValue); Serial.print (",");
    Serial.print(temperature); Serial.print (",");
    Serial.print(humidity); Serial.print (",");
    Serial.print(PM2_Value); Serial.print (",");
    Serial.print(PM10_Value); Serial.print (",");
    Serial.println (AQIColour);
#endif
  }
}

#ifdef OELCD_OP
void displaySetupScreen()
{
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.print(SplashScreen);
  display.display();
  delay(2000);
  display.clearDisplay();
}

void displayReading(int aa, int bb, int cc, char* dd, char* ee, String ff) {
  display.clearDisplay();

  // display header detail
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); //header
  display.print(dd);

  // display footer detail
  display.setCursor(0, 56);
  display.setTextSize(1);
  display.print(ee);

  // display type & units
  display.setTextSize(1);
  display.setCursor(28, 13);
  display.write(248);
  display.setCursor(34, 16);
  display.print("C");

  display.setCursor(28, 37);
  display.print("%rH");

  display.setCursor(105, 37);
  display.print("CO2");
  display.setCursor(105, 45);
  display.print("ppm");
  display.setCursor(0, 56);


  // display reading
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(bb);  //temp
  display.setCursor(0, 37);
  display.print(cc);  //humidity

  display.setCursor(55, 16);
  display.print(ff);  //AQIColour

  display.setCursor(55, 37);
  display.print(aa);  //CO2 Value

  display.display();

}
#endif

void getT9602data() //gets temp and hum values from T9602 over I2C
{
  byte aa, bb, cc, dd;
  Wire.beginTransmission(0x28);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(0x28, 4);
  aa = Wire.read();
  bb = Wire.read();
  cc = Wire.read();
  dd = Wire.read();


  // humidity = (rH_High [5:0] x 256 + rH_Low [7:0]) / 16384 x 100
  humidity = (float)(((aa & 0x3F ) << 8) + bb) / 16384.0 * 100.0;
  humidity = round(humidity);
  // temperature = (Temp_High [7:0] x 64 + Temp_Low [7:2]/4 ) / 16384 x 165 - 40
  temperature = (float)((unsigned)(cc  * 64) + (unsigned)(dd >> 2 )) / 16384.0 * 165.0 - 40.0;
  temperature = round(temperature * 10);
  temperature = temperature / 10;

}

void getT6713data()  //gets CO2 ppm value from T6700 series over I2C bus
{
  // start I2C
 
  Wire.beginTransmission(ADDR_6713);
  Wire.write(0x04); Wire.write(0x13); Wire.write(0x8B); Wire.write(0x00); Wire.write(0x01);
  // end transmission
  Wire.endTransmission();
  // read report of current gas measurement in ppm
  delay(1);
  Wire.requestFrom(ADDR_6713, 6);    // request 4 bytes from slave device
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  //  Serial.print("Func code: "); Serial.println(func_code);
  //  Serial.print("byte count: "); Serial.println(byte_count);
  //  Serial.print("MSB: ");  Serial.print(data[2]); Serial.print("  ");
  //  Serial.print("LSB: ");  Serial.print(data[3]); Serial.print("  ");
  CO2ppmValue = ((data[2] & 0x3F ) << 8) | data[3];
  //Serial.print(ppmValue);
  //return ppmValue;
}

void SWM_PM_SETUP() // sets up the input pins   for 01C dust sensor
{
  pinMode(PM10_IN_PIN, INPUT);
  pinMode(PM2_IN_PIN, INPUT);
  pinMode(PM10_OUT_PIN, OUTPUT);
  pinMode(PM2_OUT_PIN, OUTPUT);
}



void CalculateDustValue() //calculates the dust value from the stored 
                          // sample times accumulated by the interupts.
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unPM10_In;
  static uint16_t unPM2_In;
  static uint16_t unPM10_Time;
  static uint16_t unPM2_Time;
  // local copy of update flags
  static uint8_t bUpdateFlags;
  static long    PM2_Output[15];
  static long    PM10_Output[15];


  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if (bUpdateFlags & PM10_FLAG)
    {
      unPM10_In = unPM10_InShared;
      unPM10_Time = (unPM10_Time + unPM10_In);
    }

    if (bUpdateFlags & PM2_FLAG)
    {
      unPM2_In = unPM2_InShared;
      unPM2_Time = (unPM2_Time + unPM2_In);
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unPM10_In and unPM2_In, the shared
  // variables unPM10_InShared, unPM2_InShared are always owned by
  // the interrupt routines and should not be used in loop

  sampletime = millis();  //resets timer before printing output
  PM2_Output[SampleCount] = unPM2_Time ;
  PM10_Output[SampleCount] = unPM10_Time ;
  unPM2_Time = 0;
  unPM10_Time = 0;
  
  PM2_Output[0] = PM2_Output[1] + PM2_Output[2] + PM2_Output[3] + PM2_Output[4] + PM2_Output[5] + PM2_Output[6] + PM2_Output[7] + PM2_Output[8]+ PM2_Output[9]+ PM2_Output[10]+ PM2_Output[11]+ PM2_Output[12];
  PM10_Output[0] = PM10_Output[1] + PM10_Output[2] + PM10_Output[3] + PM10_Output[4] + PM10_Output[5] + PM10_Output[6] + PM10_Output[7] + PM10_Output[8] + PM10_Output[9] + PM10_Output[10] + PM10_Output[11] + PM10_Output[12];

//  Serial.print (PM2_Output[0]); Serial.print("\t");
//  Serial.print (PM10_Output[0]); Serial.println("\t");


  /* converts LP outputs to values, calculate % LPO first, then converet to µg/m3 assuming conversion is linear
              output (µS)                           concentration change (250 or 600)
     -----------------------------------    x 100 x ---------------------------------  + offset (0 or 250)
     sample rate (mS) x 1000 x NoOfSamples               percentage change (3 0r 7)

  */
  if (PM2_Output[0] / (samplerate * NoOfSamples * 10 ) >= 3 || PM10_Output[0] / (samplerate * NoOfSamples * 10 ) >= 3);
  {
    PM2_Value = round((float)PM2_Output[0] / (samplerate * NoOfSamples * 10 ) * 600 / 7 + 250);
    PM10_Value = round((float)PM10_Output[0] / (samplerate * NoOfSamples * 10 ) * 600 / 7 + 250);
  }
  {
    PM2_Value = round((float)PM2_Output[0] / (samplerate * NoOfSamples * 10 ) * 250 / 3);
    PM10_Value = round((float)PM10_Output[0] / (samplerate * NoOfSamples * 10 ) * 250 / 3);
  }
  bUpdateFlags = 0;  //reset flags and variables

// Serial.print (PM2_Output[SampleCount]); Serial.print("\t");

  if (SampleCount >= NoOfSamples)
  {
    SampleCount = 1;
//    Serial.print (PM2_Output[0]); Serial.print("\t");Serial.println("\t");
  }
  else
  {
    SampleCount++;
  }

  // Colour Values based on US EPA Air Quality Index for PM 2.5 and PM 10
  if (PM2_Value <= 12 && PM10_Value <= 54)
  {
    AQIColour = "Green ";
  }
  else if (PM2_Value <= 35 && PM10_Value <= 154)
  {
    AQIColour = "Yellow";
  }
  else if (PM2_Value <= 55 && PM10_Value <= 254)
  {
    AQIColour = "Orange";
  }
  else if (PM2_Value <= 150 && PM10_Value <= 354)
  {
    AQIColour = " Red  ";
  }
  else if (PM2_Value <= 250 && PM10_Value <= 424)
  {
    AQIColour = "Purple";
  }
  else {
    AQIColour = "Maroon";
  }
}

// simple interrupt service routine
void calcPM10()
{
  // if the pin is low, its a falling edge of the signal pulse, so lets record its value
  if (digitalRead(PM10_IN_PIN) == LOW)
  {
    ulPM10_Start = micros();
  }
  else
  {
    // else it must be a rising edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the falling and rising edges i.e. the pulse duration.
    unPM10_InShared = (uint16_t)(micros() - ulPM10_Start);
    // use set the PM10_ flag to indicate that a new PM10_ signal has been received
    bUpdateFlagsShared |= PM10_FLAG;
  }
}

void calcPM2()
{
  if (digitalRead(PM2_IN_PIN) == LOW)
  {
    ulPM2_Start = micros();
  }
  else
  {
    unPM2_InShared = (uint16_t)(micros() - ulPM2_Start);
    bUpdateFlagsShared |= PM2_FLAG;
  }
}



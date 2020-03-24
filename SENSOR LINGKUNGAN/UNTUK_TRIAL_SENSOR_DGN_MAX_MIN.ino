#include <math.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


#define DBG_OUTPUT_PORT Serial
#define Addr 0x4A

#define WindSensorPin2 (25) // The pin location of the anemometer sensor hitam
#define WindSensorPin (14) // The pin location of the anemometer sensor putih

const unsigned long eventinterval = 5000; // 1000 milisecond
// const unsigned long eventinterval2 = 5000; // 5000 milisecond
unsigned long previousTime = 0;

volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
volatile unsigned long Rotations;
volatile unsigned long  Rotations2;
volatile unsigned long wind1;
volatile unsigned long wind2;

ThreeWire myWire(16,15,17);       // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

const int numReadings = 6;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
unsigned long total = 0;                  // the running total
unsigned long average;                    // the average
unsigned long maks = 0;
unsigned long minim = 200000;

float WindSpeed;                // Wind speed (m/s)
float WindSpeed2;


const int chipSelect = 13;
File myFile; //SD CARD
static bool hasSD = false;

//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Fungsi Timer Interrupt----------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
wind1 = Rotations;
wind2 = Rotations2;
Rotations = 0;
Rotations2 = 0;
}

void setup()
{
//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Start Serial--------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
Serial.begin(9600);
//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Timer Interrupt-----------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
timer = timerBegin(0, 80, true);
timerAttachInterrupt(timer, &onTimer, true);
timerAlarmWrite(timer, 1000000, true);
timerAlarmEnable(timer);
//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Start RTC-----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
    Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();

    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);
    }

    if (Rtc.GetIsWriteProtected())
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
//--------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------Start SDCard-----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
Serial.println("Pengecekan SD Card...");
if (!SD.begin(13)) {
Serial.println("Kartu SD Gagal");}
else{
Serial.println("Kartu SD Terpasang");}
//--------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------Start Radiasi-----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
  Wire.begin();

//--------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------Start Anemometer-----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
pinMode(WindSensorPin, INPUT_PULLUP); //kena level shifter
pinMode(WindSensorPin2, INPUT_PULLUP); //kena level shifter
// pinMode(Addr, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, CHANGE);
attachInterrupt(digitalPinToInterrupt(WindSensorPin2), isr_rotation2, FALLING);
}
 
void loop()
{
radiasi();
kec1();
kec2();
unsigned long currentTime = millis();
if (currentTime - previousTime >= eventinterval){
  // radiasi();
//--------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------Serial Print-----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
RtcDateTime now = Rtc.GetDateTime();
Serial.print(" Put. Putih : "); Serial.print(wind1); Serial.print("\t"); Serial.print("Kec. Angin Putih : "); Serial.print(WindSpeed); Serial.print("\t");
Serial.print("Put. Hitam : "); Serial.print(wind2); Serial.print("\t"); Serial.print("Kec. Angin Hitam : "); Serial.print(WindSpeed2); Serial.print("\t");
Serial.print("Rad. :"); Serial.print(average); Serial.print("\t"); Serial.print("Maks. Rad."); Serial.print(maks); 
Serial.print("\t"); Serial.print("Min. Rad."); Serial.println(minim);

//----------------------------------------------------------Tulis SDCard-----------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------//
Serial.println("Menuliskan di SD Card...");
Serial.println("");
//********************************************************************************************************************************************//
myFile = SD.open("/datalog.txt", FILE_APPEND); //orinya file_write
printDateTime(now);
myFile.print(";");
myFile.print(wind1);  myFile.print(";"); myFile.print(WindSpeed);  myFile.print(";");
myFile.print(wind2);  myFile.print(";"); myFile.print(WindSpeed2);  myFile.print(";");
myFile.print(average); myFile.print(";"); myFile.print(maks); myFile.print(";"); myFile.println(minim); 
myFile.close();

minim=200000;
maks=0;

previousTime = currentTime;
}
}


//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------ISR 1---------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
void isr_rotation () {
Rotations++;
}

//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Rumus Kec1---------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
void kec1(){
if( wind1 == 0 ){
WindSpeed = wind1;
}
else if( wind1 > 0) {
WindSpeed = (wind1 * 0.0875) + 0.1;
}
}

//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------ISR 2---------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
void isr_rotation2 () {
if ((millis() - ContactBounceTime) > 20 ) { // debounce the switch contact. default = 15 ===> semakin kecil semakin sensitif
Rotations2++; 
ContactBounceTime = millis();
}
}

//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------Rumus Kec2---------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
void kec2(){
if( wind2 == 0 ){
WindSpeed2 = wind2;
}
else if( wind2 > 0) {
WindSpeed2 = (wind2 * 0.0875) + 0.1;
}
}
//--------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------Measure Radiasi---------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
void radiasi(){
  // Wire.begin();
  Wire.beginTransmission(Addr);
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.endTransmission();
  unsigned int data[2];
  Wire.beginTransmission(Addr);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);
 
  // Read 2 bytes of data luminance msb, luminance lsb
  if (Wire.available() == 2)
  {
  data[0] = Wire.read();
  data[1] = Wire.read();
  }
 
  // Convert the data to lux
  int exponent = (data[0] & 0xF0) >> 4;
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
  float luminance = pow(2, exponent) * mantissa * 0.045;


//*********** MOVING AVERAGE SENSOR RADIASI **************
  total = total - readings[readIndex];
  readings[readIndex] = luminance;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;

  if (average > maks){
  maks = average;
  }
  if (average < minim){
  minim = average;
}
}



//--------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------RTC-----------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------//
#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
    myFile.print(datestring);
}

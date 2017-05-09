
#include <EEPROM.h> // To hold running totals of cost and operating time
#include <StopWatch.h>  //Handy elapsed timer - saves some faffing with millis directly
#include <BlynkSimpleEthernet.h> // See http://docs.blynk.cc/
#define BLYNK_PRINT Serial
#include <SPI.h>
#include <Ethernet.h>
#include <SimpleTimer.h> //For calling functions outside of main loop
#include <OneWire.h> //For temp sensor
#include <DallasTemperature.h> //Ditto
#include "EmonLib.h" //For current calculations
#include "math.h"
#include "avr/eeprom.h"
             



char auth[] =    "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"; 

const int resetStatusLED = 2;  //Indicates alarms are armed/alarmcounters == 0
const int waterSensorPinA = 3;  //Water sensor input signal (sensor in washing machine pan)
const int waterSensorPinB = 4; //Second sensor to detect water around shutoff/drain area
const int washerSensorPin = A0; //Washing machine current sensor input signal 
const int dryerSensorPin = A1; //Dryer current sensor input signal
const int washerRunningButton = 5; //With dryerRunningButton resets washer/dryer cost totals and returns cyclecounters to 0
//When pressed alone will trigger notification to buy laundry detergent
const int dryerRunningButton = 6; //See above.  When pressed alone will trigger notification to buy dryer sheets
const int washerCycleCompleteLED = A2; // Indicates washer cycle complete, integrated with buy laundry detergent button
const int dryerCycleCompleteLED = A3; // Indicates dryer cycle complete, integrated with buy dryer sheets button
const int waterAlarmLED = A4;  // Indicates water alarm has activated
const int dryerAlarmLED = A5; // Indicates dryer alarm has activated
const int waterAlarmTest = 7;  //Test button for water alarm
const int dryerAlarmTest = 8; //Test button for dryer alarm
const int ventTempSensor = 9;  //18B20 One Wire temp sensor for dryer vent
const int connectedLED = 13; //Blinky light to indicate connection to Blynk server
const int alarmReset = 12;  //sets alarmcounter back to zero;
                            //Allows multiple tests before automatic alarmcounter reset (every ten minutes)
const int alarmBuzzer = 11; //for beeping when physical buttons are pressed
const int alarmPin1 = 23; //for directly connected alarm sound
int washercounter; //Keeps track of where in washing cycle washer is
int dryercounter; //Same for dryer - not really needed for one cycle dryer except to confirm readings
int ventcounter; //for taking two readings of vent temp before triggering alarm - was getting some spurious triggers when only one reading taken
int dryeralarmcounter = 0;  //Stops multiple alarm notifcations
int wateralarmcounter = 0;  //Stops multiple alarm notifcations
//unsigned long washTime; //Timer for washer cycle = used to calculate cost
unsigned long previousDryerTime = 0; //used to confirm current reading
unsigned long previousWasherTime = 0;//ditto
unsigned long previousVentTime = 0;//used to confirm vent temp reading (with ventcounter)
//unsigned long currentTime = millis();
int currentConfirmTime = 2500; //used to confirm current reading
int extendedConfirmTime = 10000; //used in washer cycle to account for extended pauses 
int ventAlarmConfirmTime = 1000;
int resetWasherTime = 25000;
float totalWasherPower; //sum of power readings during wash cycle
float totalDryerPower;// ditto for dryer
float washerTotalCost;//approx cost of washer cycle
unsigned long washerRTS; // washer run time in seconds - same as StopWatch elapsed so not really needed
unsigned long dryerRTS; // ditto for dryer
int dryerRTM; // run time in minutes
int washerRTM; //ditto
int washerRTH; //ditto
int dryerRTH; //ditto
int washerTTGS; //for time to go calculation
int washerTTGMDisplay; //for minutes time to go display
int washerTTGSDisplay; // for seconds time to go display
int dryerRTMDisplay; // for dryer run time minutes display
int dryerRTHDisplay; // for dryer runtime minutes display
int washerRTMDisplay; //same as above for washer
int washerRTHDisplay; //ditto
unsigned long washerRunTotMDisplay; //for running total minute display
int washerRunTotHDisplay; //for running total hours display
unsigned long dryerRunTotMDisplay; // same as above for dryer
int dryerRunTotHDisplay; //diito
float runningWasherCost; //for running total washer cost
long runningWasherCostint; //convert to long int for EEPROM
float runningDryerCost; // for running total dryer cost
long runningDryerCostint; //convery to long int for EEPROM
long runningWasherTime; //for running total washer time
long runningDryerTime; // for running total dryer time
float dryerTotalCost;//approx cost of dryer cycle
float kWhCost = 0.08;// Utility rate for H1 2017 is $0.08/kWh
int washSample;// used to calculate average power during washer cycle
int drySample;//ditto for dryer
float averageWasherPower;//see above
float averageDryerPower;//see above
int washerCycle;//used to rotate through time and cost displays
int dryerCycle;//ditto
float totalPower; // for Blynk guage display - aggregate of washer and dryer power in kW
float washerCostPlug = 0.85; // arbitrary number for detergent cost and hot water not heated in machine
int washPlugFactor; //cycles between 1 and 0 - quick way to zero out plug cost each cycle start
int dryPlugFactor; //ditto
float dryerCostPlug = 0.05; //see above
int washerLEDState = LOW;  //For keeping track of detergent supplies toggle
int washerButtonReading;  //ditto
int dryerLEDState = LOW;  //ditto for dryer sheets
int dryerButtonReading;  //ditto
String detergentStatus = String("Detergent OK"); // for showing detergent status in Blynk app
String dryerSheetStatus = String("Dryer Sheets OK"); // ditto for dryer
String costUnit = "$"; //not sure why I bothered to declare this - could concatenate directly
String sigma = "∑ "; //running totals indicator
unsigned long time;  //for buttonHold detect - since same buttons are used for different things
int buttonHold = 1000;  //ditto
byte runningWasherTimeEEPROMAddress=0;  //and 1,2,3 because long 
byte runningWasherCostEEPROMAddress=4; //and 5,6,7 because will convert float to long
byte runningDryerTimeEEPROMAddress=8; //and 9,10,11
byte runningDryerCostEEPROMAddress=12; // and 13, 14, 15  



WidgetBridge bridge1(V1); //Initiating Bridge Widget on V1 of Washer/Dryer Arduino -- 
                          // Plays and stops alarm sounds and lights on School Bus Raspi (WasherDryerAlarm1.py)

OneWire oneWire (ventTempSensor); // Instantiate vent temp sensor object. Nice tutorial for oneWire use at https://www.youtube.com/watch?v=1GD29sXLOJ0
DallasTemperature sensors(&oneWire); // ditto

SimpleTimer timer; //Instantiate timer object



EnergyMonitor emon1;  //Instantiate energy monitor for washer https://learn.openenergymonitor.org/electricity-monitoring/ct-sensors/how-to-build-an-arduino-energy-monitor-measuring-current-only?redirected=true
EnergyMonitor emon2;  //Instantiate energy monitor for dryer

StopWatch washerTTG(StopWatch::SECONDS); //instantialte SW objects
StopWatch dryerRunTimeSW(StopWatch::SECONDS);
StopWatch washerRunTimeSW(StopWatch::SECONDS);


void notifyOnWaterDetectionAlarm()  //wisott
{
 
   if (digitalRead(waterSensorPinA) == HIGH || digitalRead(waterSensorPinB) == HIGH || digitalRead(waterAlarmTest) == LOW) 
  
  {  
    // Test does NOT test sensors themselves - just alarm code and hardware
    
     if (wateralarmcounter == 0)  //counter stops multiple notification/possible flood (ha!) error
       
       {
        bridge1.digitalWrite(25, LOW); //to raspi3
        //Blynk.virtualWrite(V11, "OFF");//Blynk alarm status LED
        Blynk.virtualWrite(V5, 255); //Blynk app water alarm LED
        digitalWrite(waterAlarmLED, HIGH); //panel alarm LED
        digitalWrite(alarmPin1, HIGH);
        //digitalWrite(alarmPin2, HIGH);
        digitalWrite(resetStatusLED, LOW); //panel alarm status LED
        Blynk.notify("Water Detected in Laundry Room!"); //Blynk notification
        Serial.println("Water Detected in Laundry Room!");
        wateralarmcounter = 1;
        }
   }

   if (digitalRead(waterAlarmTest) == HIGH && wateralarmcounter == 1 && digitalRead(waterSensorPinA) == LOW && digitalRead(waterSensorPinB) == LOW) 
  
     {
      bridge1.digitalWrite(25, HIGH);
      digitalWrite(alarmPin1, LOW); 
      Serial.println("Water Alarm silenced");
      wateralarmcounter = 2;
      digitalWrite(resetStatusLED, LOW);   
      }
 }


void resetAlarms() //resets alarmcounter and physical/virtual LEDs
{ 
  
   if (digitalRead(alarmReset) == LOW)  
   {
    wateralarmcounter = 0;
    dryeralarmcounter = 0;
    ventcounter = 0;
    digitalWrite(resetStatusLED, HIGH);
    Blynk.virtualWrite(V5, 0);
    Blynk.virtualWrite(V6, 0);
    digitalWrite(dryerAlarmLED, LOW);
    digitalWrite(waterAlarmLED, LOW);
    digitalWrite(alarmPin1, LOW);
    Serial.println("Alarms status reset");  
   }
}

void autoAlarmsReset() // resets alarm counter after 10 minutes to avoid inadvertantly supressing alarms

 {
  wateralarmcounter = 0; 
  dryeralarmcounter = 0;
  ventcounter = 0;
  digitalWrite(resetStatusLED, HIGH);
  Blynk.virtualWrite(V5, 0);
  Blynk.virtualWrite(V6, 0);
  digitalWrite(dryerAlarmLED, LOW);
  digitalWrite(waterAlarmLED, LOW);
  digitalWrite(alarmPin1, LOW);
  Serial.println("Alarms status  auto reset");  
 }
 
void getVentTemp()  //monitors dryer exhaust temp at dryer outlet via Blynk widget and notifies on high temp
 
{
  unsigned long currentTime = millis();  
  sensors.requestTemperatures();
  float currentTemp;
  currentTemp = sensors.getTempCByIndex(0);
  Serial.print("Temp = ");
  Serial.print("\t");
  Serial.println(currentTemp);
  String tempString = String(currentTemp,1);
  String tempUnit = "°C";
    
  if (currentTemp == -127.00)
  
  {
    Blynk.virtualWrite(V9, "Error");
  }
  
  else
  
  {
    Blynk.virtualWrite(V9, tempString + tempUnit);
  }
  
  if (currentTemp >80 || digitalRead(dryerAlarmTest) == LOW) 
  
  {
    Serial.println (millis());
    Serial.println(currentTime);
    previousVentTime = currentTime;
    Serial.println(previousVentTime);
    Serial.println("Confirming high dryer vent temp");
    ventcounter = 1;       
  }
  
  if (currentTemp >80 && ventcounter == 1 && currentTime - previousVentTime >= ventAlarmConfirmTime || digitalRead(dryerAlarmTest) == LOW )
 
  {
  
    if (dryeralarmcounter == 0) //counter stops multiple notification/possible flood error

    {  
     Blynk.notify ("High Dryer Vent Temperature! FIRE RISK: CHECK IMMEDIATELY");
     bridge1.digitalWrite(13, LOW);
     digitalWrite(alarmPin1, HIGH);
     digitalWrite(dryerAlarmLED, HIGH);
     digitalWrite(resetStatusLED, LOW);
     Blynk.virtualWrite(V6, 255);
     dryeralarmcounter = 1;
     Serial.println("High dryer vent temp!");
     Serial.print("Temp = ");
     Serial.print("\t");
     Serial.println(currentTemp); 
     Serial.println(dryeralarmcounter);
     ventcounter = 0;
    }  
  } 
  
  if (digitalRead(dryerAlarmTest) == HIGH && dryeralarmcounter == 1 && currentTemp <80)
    
    {
     bridge1.digitalWrite(13, HIGH);
     digitalWrite(alarmPin1, LOW); 
     Serial.println("Dryer alarm silenced");
     dryeralarmcounter = 2;
     ventcounter = 0;
    }
}

void setBlinkOn()  //Blinky physical and virtual LEDs for connection status

 {

  if (Blynk.connected() == true);
  
    {
    Blynk.virtualWrite(V2, 255); //Blynk server connected LED
    digitalWrite(connectedLED, HIGH);
    }
 }
  
void setBlinkOff()

  {
  Blynk.virtualWrite(V2, 0);
  digitalWrite(connectedLED, LOW);
  }


void washerDryerMonitor()

{  
   double washerCurrent = emon1.calcIrms(1480);
   Serial.print("Washer Current = ");
   Serial.print("\t");
   Serial.println(washerCurrent);
   float washerPower =  (0.85* washerCurrent * 120)/1000;    //P(kW) = PF × I(A) × V(V) / 1000
   Serial.print("Washer Power = ");
   Serial.print("\t");
   Serial.print(washerPower);
   Serial.println("kW");
   double dryerCurrent = emon2.calcIrms(1480);
   Serial.print("Dryer Current = ");
   Serial.print("\t");
   Serial.println(dryerCurrent);
   float dryerPower = (0.95 * dryerCurrent * 240)/1000;    //P(kW) = PF × I(A) × V(V) / 1000
   Serial.print("Dryer Power = ");
   Serial.print("\t");
   Serial.println(dryerPower);
   Serial.println("kW");
   totalPower = washerPower + dryerPower;
   String powerString = String(totalPower,2);
   String powerUnit = String("kW");
   Blynk.virtualWrite(V16, powerString + powerUnit); //Blynk gauge widget does not display units
   totalWasherPower = washerPower + totalWasherPower;
   Serial.print("TotalWasherPower = ");
   Serial.println(totalWasherPower);
   averageWasherPower = totalWasherPower/washSample; // timer fires every 5s
   Serial.print("Average Washer Power = ");
   Serial.println(averageWasherPower);
   Serial.print("washSample = ");
   Serial.println(washSample);
   washerRTS = washerRunTimeSW.elapsed();
   washerRTM = washerRTS/60;
   washerRTH = washerRTM/60;
   Serial.print("washerRunTimeSW.elapsed/60 = ");
   Serial.println(washerRTM);
   Serial.print("washerRTM/60 = ");
   Serial.println(washerRTH);
   washerTotalCost = (averageWasherPower * kWhCost * washerRTM/60) + (washerCostPlug * washPlugFactor); 
   Serial.print("washerTotalCost = ");
   Serial.println(washerTotalCost);
   String washCostString = String(washerTotalCost,2);
   totalDryerPower = dryerPower + totalDryerPower;
   Serial.print("TotalDryerPower = ");
   Serial.println(totalDryerPower);
   averageDryerPower = totalDryerPower/drySample;
   Serial.print("Average Dryer Power = ");
   Serial.println(averageDryerPower);
   Serial.print("drySample = ");
   Serial.println(drySample);
   dryerRTS = dryerRunTimeSW.elapsed();
   dryerRTM = dryerRTS/60;
   dryerRTH = dryerRTM/60;
   Serial.print("dryerRunTimeSW.elapsed/60 = ");
   Serial.println(dryerRTM);
   Serial.print("dryerRTM/60 = ");
   Serial.println(dryerRTH);
   Serial.print("kWhCost =");
   Serial.println(kWhCost);
   dryerTotalCost = (averageDryerPower * kWhCost * dryerRTM /60) + (dryerCostPlug * dryPlugFactor); 
   Serial.print("dryerTotalCost = ");
   Serial.println(dryerTotalCost);
   String dryCostString = String(dryerTotalCost,2);
   unsigned long currentTime = millis();
   washerTTGS = 705 - washerTTG.elapsed();// used separate calculation and display variables to show time in hours/mins/seconds
   washerTTGMDisplay = washerTTGS/60;
   washerTTGSDisplay = washerTTGS % 60;
   String hours = String("h:");
   String minutes = String("m:");
   String seconds = String("s");
   String TTGM = String(washerTTGMDisplay);
   String TTGS = String(washerTTGSDisplay);
   dryerRTMDisplay = dryerRTM % 60;
   dryerRTHDisplay = dryerRTH;
   String dryerRunTime = String(dryerRTHDisplay + hours + dryerRTMDisplay + minutes);
   washerRTMDisplay = washerRTM % 60;
   washerRTHDisplay = washerRTH;
   String washerRunTime = String(washerRTHDisplay + hours + washerRTMDisplay + minutes);
   washerRunTotMDisplay = runningWasherTime % 60;
   washerRunTotHDisplay = runningWasherTime/60;
   String runningTotWasherRunTime = String(sigma + washerRunTotHDisplay + hours + washerRunTotMDisplay + minutes);
   String runningTotWasherCost = String(sigma + costUnit + runningWasherCost);
   dryerRunTotMDisplay = runningDryerTime % 60;
   dryerRunTotHDisplay = runningDryerTime/60;
   String runningTotDryerRunTime = String(sigma + dryerRunTotHDisplay + hours + dryerRunTotMDisplay + minutes);
   String runningTotDryerCost = String(sigma + costUnit + runningDryerCost);
    
    if (washerCurrent >= 0.08 && washercounter == 0 && currentTime >120000 && digitalRead(waterAlarmTest) == HIGH && digitalRead(dryerAlarmTest) == HIGH)
  // alarm test conditions are because sometimes testing triggered washer start - not sure why
     {
      Serial.println("Confirming Washer Start Current");
      Serial.println (millis());
      Serial.println(currentTime);
      previousWasherTime = currentTime;
      Serial.println(previousWasherTime);
      washercounter = 1;    
     }
      
    if  (washerCurrent >= 0.08 && washercounter == 1 && currentTime - previousWasherTime >= currentConfirmTime && digitalRead(waterAlarmTest) == HIGH && digitalRead(dryerAlarmTest) == HIGH)    
    
   {
    washerCycle = 4; //for display rotation - cycles from 4 to 7. Modulo calc not needed - legacy from earlier version when this tracked something else (also hence the non-descriptive name)
    washPlugFactor = 0;
    digitalWrite (washerCycleCompleteLED, LOW);
    Blynk.virtualWrite(V10, 255);  // washer filling indicator
    Blynk.virtualWrite(V3, 255);// washer washing indicator
    Blynk.virtualWrite(V7, 0); // wash cycle done indicator
    Serial.println("Washer cycle running -- filling for wash");
    washerTTG.reset();
    washerRunTimeSW.reset();
    Blynk.virtualWrite(V11, "N/A UNTIL RINSE CYCLE");
    washerRunTimeSW.start();
    Serial.println("Washer timer is running");
    totalWasherPower = 0.001;
    washPlugFactor = 1;
    washSample = 2;
    washercounter = 2;   
   }
    
   else if (washerCurrent < 0.08 && washercounter == 1) //the "two-step" current readings deal with possible false starts because fill current is so low and also to take care of times during wash cycle when
   //washer is in a pause state as it moves from one activity to another (e.g. from stopping filling to starting washer motor or between stopping washing and starting empyting)
   
    {
      washercounter = 0;
      //washSample = 1;
    }
  
 if (washercounter == 2 || washercounter == 4 || washercounter == 6 || washercounter == 8) //increments washSample while running for power and cost calcs 
 
   {
     washSample++;
   }

 if (washerCycle % 4 == 0) // for display rotations
     {
     Blynk.virtualWrite(V15, costUnit + washCostString);
     washerCycle ++;
     }
     else if (washerCycle % 4 == 1)
     {
       Blynk.virtualWrite(V15, washerRunTime);
       washerCycle ++;
     }
     else if (washerCycle % 4 == 2)
     {
       Blynk.virtualWrite(V15, runningTotWasherCost);
       washerCycle ++;
     }
     else if (washerCycle % 4 == 3)
     {
       Blynk.virtualWrite(V15, runningTotWasherRunTime);
       washerCycle = 4;
     }
       
    
  if  (washerCurrent <= 0.08 && washercounter == 2 && currentTime - previousWasherTime >= resetWasherTime) // another check for false starts. 
 //If a washer motor level current is not detected within 15 minutes after fill start monitor will reset
 
   {
    digitalWrite (washerCycleCompleteLED, HIGH);
    Blynk.virtualWrite(V10, 0);  // washer filling indicator
    Blynk.virtualWrite(V7, 255); // wash cycle done indicator
    Blynk.virtualWrite(V3, 0);
    Serial.println("Washer cycle running false start -resetting");
    washercounter = 0;
    washSample = 1;
    washerRunTimeSW.stop();
    washerRunTimeSW.reset();
    totalWasherPower = 0.001;
    washerCycle = 4;
   }    
  
  if (washerCurrent >= 2.0 && washercounter == 2) 
   
   {
    Serial.println (millis());
    Serial.println(currentTime);
    previousWasherTime = currentTime;
    Serial.println(previousWasherTime);
    Serial.println("Confirming washer washing current");
    washercounter = 3;
    washSample++;
   }
 
  if (washerCurrent >= 2.0 && washercounter == 3 && currentTime - previousWasherTime >= currentConfirmTime)
  
    {
     digitalWrite (washerCycleCompleteLED, LOW);
     Blynk.virtualWrite(V3, 255);
     Blynk.virtualWrite(V10, 0); 
     //Blynk.virtualWrite(V3, 255);
     //Blynk.virtualWrite(V11, "Armed");
     Serial.println("Washer cycle running -- washing");
     washercounter = 4;
     washSample++;
     }

   else if (washerCurrent <2.0 && washercounter == 3)
    {
      washercounter = 2;
    }
  
  if (washerCurrent <= 0.5 && washercounter == 4)
  
  {
    Serial.println(currentTime);
    previousWasherTime = currentTime;
    Serial.println(previousWasherTime);
    Serial.println("Confirming washer filling for rinse current");
    washercounter = 5;
    washSample++;     
  }
  
    if (washerCurrent <= 0.5 && washercounter == 5 && currentTime - previousWasherTime >= extendedConfirmTime)
 
    { 
      Serial.print("currentTime = ");
      Serial.println(currentTime);
      //previousWasherTime = currentTime;
      Serial.print("previousWasherTime = ");
      Serial.println(previousWasherTime);
      Serial.println("Washer cycle runnning - filling for rinse");
      Blynk.virtualWrite(V3, 0);
      Blynk.virtualWrite(V10, 255);  // washer filling indicator
      Blynk.virtualWrite(V13, 255); //washer rinsing indicator
      Serial.println (millis());
      Serial.print("currentTime = ");
      Serial.println(currentTime);
      //previousWasherTime = currentTime;
      Serial.print("previousWasherTime = ");
      Serial.println(previousWasherTime);
      washercounter = 6;
      washSample++;
    }

     else if (washerCurrent > 2.0 && washercounter == 5)
     
    {
      washercounter = 4;
    }
     
  if (washerCurrent >= 2.0 && washercounter == 6)
      
   { 
    Serial.println(currentTime);
    previousWasherTime = currentTime;
    Serial.println(previousWasherTime);
    Serial.println("Confirming washer rinse/spin stage");
    washercounter = 7;
    washSample++;
   }
    
  if (washerCurrent >= 2.0 && washercounter == 7 && currentTime - previousWasherTime >= extendedConfirmTime)
  
   { 
    Serial.print("currentTime = ");
    Serial.println(currentTime);
    washerTTG.start();
    Serial.print("washerTTG.elapsed = ");
    Serial.println(washerTTG.elapsed());
    //previousWasherTime = currentTime;
    Serial.print("previousWasherTime = ");
    Serial.println(previousWasherTime);
    Serial.println("Washer cycle running -- rinse/spin");
    Blynk.virtualWrite(V3, 0);  //washer washing indicator
    Blynk.virtualWrite(V10, 0);  // washer filling indicator
    Blynk.virtualWrite(V13, 255); //washer rinsing indicator
    Blynk.virtualWrite(V11, TTGM + minutes + TTGS + seconds); 
    washercounter = 8;
    washSample++;
   }

  else if (washerCurrent < 2.0 && washercounter == 7)
  {
    washercounter = 6;
  }
    
  if (washercounter == 8)
   {
    Blynk.virtualWrite(V11, TTGM + minutes + TTGS + seconds);
    Serial.print("washercounter = ");
    Serial.println(washercounter); 
    Serial.print("washerTTG.elapsed = ");
    Serial.println(washerTTG.elapsed()); 
   }
     
   if (washerCurrent <= 0.2 && washercounter == 8) 
     
   {    
    Serial.println (millis());
    Serial.println(currentTime);
    previousWasherTime = currentTime;
    Serial.println(previousWasherTime);
    Serial.println("Confirming washer cycle complete");
    //Blynk.virtualWrite(V11, "00m:00s"); 
    washercounter = 9;
    washSample++;
   }
 
  if (washerCurrent <= 0.2 && washercounter == 9 && currentTime - previousWasherTime >= extendedConfirmTime)

   {
    Serial.print("currentTime = ");
    Serial.println(currentTime);
    Serial.print("previousWasherTime = ");
    Serial.println(previousWasherTime);
    washerTTG.stop();
    washerTTG.reset();
    washerRunTimeSW.stop();
    runningWasherCost = runningWasherCost + washerTotalCost;
    runningWasherCostint = (runningWasherCost*100);
    EEPROMWritelong(runningWasherCostEEPROMAddress, runningWasherCostint);
    runningWasherCostEEPROMAddress+=4;
    runningWasherTime = runningWasherTime +  washerRTM;
    EEPROMWritelong(runningWasherTimeEEPROMAddress, runningWasherTime);
    runningWasherTimeEEPROMAddress+=4;
    digitalWrite (washerCycleCompleteLED, HIGH);
    tone(alarmBuzzer, 3000, 2500);
    Blynk.notify("Washer Cycle Complete - put your clothes in the dryer!");
    Serial.println("Washer Cycle Complete");
    Blynk.virtualWrite(V3, 0); //Blynk washer cycle running LED
    Blynk.virtualWrite(V13, 0);
    Blynk.virtualWrite(V7, 255); //Blynk washercycle done LED
    Blynk.virtualWrite(V11, "00m:00s"); 
    Serial.print("Washer Power = ");
    Serial.print("\t");
    Serial.print(washerPower);
    Serial.println("kW");
    Serial.println(washercounter);
    Serial.print("Washer ran for ");
    Serial.print(washerRunTimeSW.elapsed() / 60 );
    Serial.println("minutes"); 
    Serial.print("Total power used by washer this cycle =  ");
    Serial.print(totalWasherPower);
    Serial.println("kW");
    Serial.print("Approx cost of washer this cycle =  $");
    Serial.println(washerTotalCost);
    washercounter = 0;
    washerCycle = 4;
   }

 else if (washerCurrent > 0.2 && washercounter == 9)
  {
    washercounter = 8;
  }
    
 // End of washer monitoring code: Start dryer monitoring  
  
 if (dryerCurrent > 2.0 && dryercounter == 0 && currentTime >60000)

   {
    
    Serial.println("Confirming Dryer Start Current");
    Serial.println (millis());
    Serial.println(currentTime);
    previousDryerTime = currentTime;
    Serial.println(previousDryerTime);
    dryercounter = 1;
    drySample ++;
    
   }

  if (currentTime - previousDryerTime >= currentConfirmTime && dryerCurrent > 2.0 && dryercounter == 1)
 
   {
      digitalWrite (dryerCycleCompleteLED, LOW);
      //dryerTimer.reset();
      //dryerTimer.start();
      dryerRunTimeSW.reset();
      dryerRunTimeSW.start();
      dryerCycle = 4;
      dryPlugFactor = 0;
      Serial.println("Dryer timer is running");
      Blynk.virtualWrite(V4, 255);
      Blynk.virtualWrite(V8, 0);
      Serial.println("Dryer monitor running");
      Serial.println (currentTime);
      Serial.println(previousDryerTime);
      drySample = 2;
      totalDryerPower = 0.001;
      dryPlugFactor = 1;
      dryercounter = 2;
      dryerCycle = 4;
      //runningDryerCost = runningDryerCost + dryerTotalCost; 
      //drySample ++;
        
    }

     else if (dryerCurrent < 2.0 && dryercounter == 1)
     
      {
        dryercounter = 0;
        drySample = 1;
      }
    
    
   if (dryercounter == 2)
   
   {
     drySample ++;
     //runningDryerCost = runningDryerCost + dryerTotalCost; 
     
   }

   
  if (dryerCycle % 4 == 0)
     {
     Blynk.virtualWrite(V14, costUnit + dryCostString);
     Blynk.virtualWrite(V17, dryerSheetStatus);
     dryerCycle ++;
     }
     else if (dryerCycle % 4 == 1)
     {
       Blynk.virtualWrite(V14, dryerRunTime);
       Blynk.virtualWrite(V17, detergentStatus);
       dryerCycle ++;
     }
     else if (dryerCycle % 4 == 2)
     {
       Blynk.virtualWrite(V14, runningTotDryerCost);
       Blynk.virtualWrite(V17, dryerSheetStatus);
       dryerCycle ++;
     }
     else if (dryerCycle % 4 == 3)
     {
       Blynk.virtualWrite(V14, runningTotDryerRunTime);
       Blynk.virtualWrite(V17, detergentStatus);
       dryerCycle = 4;
     }
       
 
  if (dryerCurrent <= 2.0 && dryercounter == 2)
  
    {
      Serial.println (millis());
      Serial.println(currentTime);
      previousDryerTime = currentTime;
      Serial.println(previousDryerTime);
      Serial.println("Confirming Dryer Stop Current");
      dryercounter = 3;
      drySample ++;   
    }
  
   if (dryerCurrent <= 2.0 && dryercounter == 3 && currentTime - previousDryerTime > currentConfirmTime) 
    
    {
      digitalWrite (dryerCycleCompleteLED, HIGH);
      Blynk.notify("Dryer Cycle Complete - don't forget to clean the filter!");
      Serial.println("Dryer Cycle Complete");
      tone(alarmBuzzer, 3000, 2500);
      dryerRunTimeSW.stop();
      //dryerLEDState = LOW;
      //digitalWrite(dryerRunningLED, dryerLEDState);
      Blynk.virtualWrite(V4, 0);
      Blynk.virtualWrite(V8, 255);  //Blynk Dryer cycle done LED
      dryercounter = 0;
      //dryerTimer.stop();
      Blynk.virtualWrite(V14, costUnit + dryCostString);
      runningDryerTime = runningDryerTime + dryerRTM;
      EEPROMWritelong(runningDryerTimeEEPROMAddress, runningDryerTime);
      runningDryerTimeEEPROMAddress+=4;
      runningDryerCost = runningDryerCost + dryerTotalCost;
      runningDryerCostint = round(runningDryerCost*100);
      EEPROMWritelong(runningDryerCostEEPROMAddress, runningDryerCostint);
      runningDryerCostEEPROMAddress+=4; 
      Serial.print("Dryer Power = ");
      Serial.print("\t");
      Serial.print(dryerPower);
      Serial.println("kW");
      Serial.println(dryercounter);
      Serial.print("Dryer ran for ");
      Serial.print(dryerRunTimeSW.elapsed() / 60 );
      Serial.println("minutes"); 
      Serial.print("Power used by dryer this cycle =  ");
      Serial.print(totalDryerPower);
      Serial.println("kW");
      Serial.print("Approx total cost of dryer this cycle =  $");
      Serial.println(dryerTotalCost);
      dryercounter = 0;
      dryerCycle = 0;
    }

     else if (dryerCurrent > 2.0 && dryercounter == 3)
      {
        dryercounter = 2;
      }
    }
    
 // end of dryer monitoring code 

void washerDryerLEDButtonMonitors() // for manual reset of cycle monitors and cycle cost/time displays (does not reset running totals)
{
  
  washerButtonReading = digitalRead(washerRunningButton);
  dryerButtonReading = digitalRead(dryerRunningButton);

   if (washerButtonReading == LOW && dryerButtonReading == LOW && millis() -time > buttonHold) //time/buttonhold tests stop inadvertant supply triggers if one button pressed slightly before the other
   {
     tone(alarmBuzzer, 3000, 1500);
     washercounter = 0;
     dryercounter = 0;
     Blynk.virtualWrite(V3, 0);
     Blynk.virtualWrite(V13, 0);
     Blynk.virtualWrite(V7, 255);
     Blynk.virtualWrite(V4, 0);
     Blynk.virtualWrite(V8, 255);
     Blynk.virtualWrite(V10, 0);
     digitalWrite (washerCycleCompleteLED, HIGH);
     digitalWrite (dryerCycleCompleteLED, HIGH);
     totalWasherPower = 0.001;
     totalDryerPower = 0.001;
     washPlugFactor = 0;
     dryPlugFactor = 0;
     //washerTotalCost = runningWasherCost;
     //dryerTotalCost = runningDryerCost;
     washerCycle = 4;
     dryerCycle = 4;
    // Blynk.virtualWrite(V15, costUnit + washCostString);
     //Blynk.virtualWrite(V14, costUnit + dryCostString);
     Serial.print("washerTotalCost after reset = ");
     Serial.println(washerTotalCost);
     Serial.print("dryerTotalCost after reset = ");
     Serial.println(dryerTotalCost);
     //dryerTotalCost = 0;
     washSample = 1;
     drySample = 1;
     //washerTimer.stop();
     washerRunTimeSW.stop();
     //dryerTimer.stop();
     dryerRunTimeSW.stop();
     //washerTimer.reset();
     washerRunTimeSW.reset();
     dryerRunTimeSW.reset();
     //dryerTimer.reset();
     washerTTG.stop();
     washerTTG.reset();  
     time = millis();
   }
     
     
   if (washerButtonReading == LOW && dryerButtonReading == HIGH && millis() -time > buttonHold) //for supply ordering notification/display
   
   {  
    time = millis();
   
     if (washerLEDState == HIGH)
     
     {
       tone(alarmBuzzer, 2000, 1000);
       washerLEDState = LOW;
       detergentStatus = String("Detergent OK");
     }
     
     else
    {
     tone(alarmBuzzer, 2000, 1000);
     washerLEDState = HIGH;
     Blynk.notify("Running low on laundry detergent -- get more next trip to grocery store");
     detergentStatus = String("Detergent Low"); 
    }
   }
   
   if (dryerButtonReading == LOW && washerButtonReading == HIGH && millis() -time > buttonHold)
   
   {  
    time = millis();
   
     if (dryerLEDState == HIGH)
   
      {
       tone(alarmBuzzer, 2000, 1000);
       dryerLEDState = LOW;
       dryerSheetStatus = String("Dryer Sheets OK");
      }
    else
    {
     tone(alarmBuzzer, 2000, 1000);
     dryerLEDState = HIGH;
     Blynk.notify("Running low on laundry detergent -- get more next trip to grocery store");
     dryerSheetStatus = String("Dryer Sheets Low");
   }
  }
} 

void setup()
{
  Serial.begin(57600); // See the connection status in Serial Monitor
  pinMode(waterSensorPinA, INPUT);
  pinMode(waterSensorPinB, INPUT);
  pinMode(washerSensorPin, INPUT);
  pinMode(dryerSensorPin, INPUT);
  pinMode(washerRunningButton, INPUT_PULLUP);
  pinMode(dryerRunningButton, INPUT_PULLUP);
  pinMode(waterAlarmLED, OUTPUT);
  pinMode(resetStatusLED, OUTPUT);
  pinMode(dryerAlarmLED, OUTPUT);
  pinMode(connectedLED, OUTPUT);
  pinMode(alarmBuzzer, OUTPUT);
  pinMode(alarmPin1, OUTPUT);
  digitalWrite(alarmPin1, LOW);
  pinMode (washerCycleCompleteLED, OUTPUT);
  pinMode (waterAlarmTest, INPUT_PULLUP);
  pinMode (dryerAlarmTest, INPUT_PULLUP);
  pinMode(alarmReset, INPUT_PULLUP);
  digitalWrite (washerCycleCompleteLED, HIGH);
  digitalWrite (connectedLED, LOW);
  pinMode (dryerCycleCompleteLED, OUTPUT);
  digitalWrite (dryerCycleCompleteLED, HIGH);
  emon1.current(washerSensorPin, 29); 
  emon2.current(dryerSensorPin, 29);
  totalWasherPower = 0.001;
  totalDryerPower = 0.001;
  dryPlugFactor = 0;
  washPlugFactor = 0;
  washSample = 1;
  drySample = 1;
  washercounter = 0;
  dryercounter = 0;
  ventcounter = 0;
  washerCycle = 4;
  dryerCycle = 4;
  digitalWrite(waterAlarmLED, LOW);
  digitalWrite(dryerAlarmLED, LOW);
  digitalWrite(resetStatusLED, HIGH);
  runningWasherTime = (EEPROMReadlong(runningWasherTimeEEPROMAddress)+1);
  runningWasherCost = (EEPROMReadlong(runningWasherCostEEPROMAddress)+0.00)/100;
  runningDryerTime = (EEPROMReadlong(runningDryerTimeEEPROMAddress)+1);
  runningDryerCost = (EEPROMReadlong(runningDryerCostEEPROMAddress)+0.00)/100;
  sensors.begin();  
  
  
  Blynk.begin(auth);  // Arduino connects to the Blynk Cloud.
  while (Blynk.connect() == false) {
   //  wait....
  }
  Blynk.virtualWrite(V3, 0); //Wash Cycle virtual LED
  Blynk.virtualWrite(V4, 0); //Dryer Cycle Virtual LED
  Blynk.virtualWrite(V5, 0); //Water Alarm Virtual LED
  Blynk.virtualWrite(V6, 0); //Dryer Alarm Virtual LED
  Blynk.virtualWrite(V10, 0);
  Blynk.virtualWrite(V13, 0);
  digitalWrite(connectedLED, HIGH);
  Blynk.virtualWrite(V7, 255);
  Blynk.virtualWrite(V8, 255);
  Blynk.virtualWrite(V11, "00m:00s");
  
  
  timer.setInterval(2000, notifyOnWaterDetectionAlarm);
  timer.setInterval(500, washerDryerLEDButtonMonitors);
  timer.setInterval (5000, washerDryerMonitor);
  timer.setInterval(339, setBlinkOn);
  timer.setInterval(667, setBlinkOff);
  timer.setInterval(1500, resetAlarms);
  timer.setInterval(600000, autoAlarmsReset);
  timer.setInterval(2000, getVentTemp);

}
 
  //Syncs LED and button states stored on server; runs every time Blynk connection is established 
  bool isFirstConnect = true;

  BLYNK_CONNECTED() {
  if (isFirstConnect) {
    // Request Blynk server to re-send latest values for all pins
  Blynk.syncAll();

   isFirstConnect = false;
 }
   bridge1.setAuthToken("xxxxxxxxxxxxxxxxxxxxxxxxxx"); // Token for Raspi3 (school bus alarms)
 }  

void loop() {

 Blynk.run(); // All the Blynk Magic happens here...
 timer.run();
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);

      }

   //This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
   
  

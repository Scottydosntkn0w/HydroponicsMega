#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DFRobot_ECPRO.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <movingAvg.h>



#define PH_PIN A1
#define EC_PIN A2
#define TE_PIN A3



char c;
String dataIn;


int n = 0;



//Digital Outputs
int lightPin = 22; //Relay1
int waterPumpPin = 23; //Relay2
int P_pump1Pin = 24; //Relay3
int P_pump2Pin = 25; //Relay4
int P_pump3Pin = 26; //Relay5
int P_pump4Pin = 27; //Relay6
int EC_enable = 28; //Relay7


//Digital Inputs
int WaterLevelPin = 44;
int WaterLevelState = 0;

//Analog Inputs
//Flow Meter
const byte interruptPin = 2; 
volatile double waterFlow = LOW;

//PH Meter
DFRobot_PH ph;
uint16_t voltagePH;
float phValue;
movingAvg phAVG(10);
//Electrical Conductivity Meter
DFRobot_ECPRO ec;
uint16_t EC_Voltage;
float Conductivity;
movingAvg ecAVG(10);
//Temperature Sensor
DFRobot_ECPRO_PT1000 ecpt;
uint16_t TE_Voltage;
float Temp;
movingAvg TempAVG(10);
//Power Meter
int POWERsensorPin = A4;
int POWERsensorValue = 0;


//# Desired range for electrical conductivity
int range_ec_high = 1300;
int range_ec_low = 1000;
//# Desired range for pH
int range_ph_high = 6.4;
int range_ph_low = 5.5;
//# pH range that will immediately cause a pH warning
int range_ph_high_danger = 7.0;
int range_ph_low_danger = 5.0;

unsigned long startTime = millis();
unsigned long ECReadDelay = 60000;
unsigned long waterADJDelay = 300000;//300000;
unsigned long timeOfADJ = millis();
unsigned long timeOfRead = millis();
unsigned long DurationSinceECREad = 0;
unsigned long DurationSinceADJ = 0;
unsigned long DurationSinceSerialSend = 0;
unsigned long SerialSendDelay = 30000;
unsigned long timeOfSerialSend = millis();
String Mode;
String prev_Mode;
int Temp20 = 8;
float PHoffset = -2.85;


String manualModeLoop(String prev_Mode)
{
  if (prev_Mode == "Auto")
  {
    prev_Mode = "Manual";
  }
  return prev_Mode;
}

void SerialtoDigitalWrite(String Val, int PinNumber)
{
  if (Val == "1"){digitalWrite(PinNumber,HIGH);}
  if (Val == "0"){digitalWrite(PinNumber,LOW);}
  Serial1.println("sensor/WaterLevel: "+ String(WaterLevelState));
  Serial1.println("output/Light/state: "+ String(digitalRead(lightPin)));
  Serial1.println("output/Water Pump/state: "+ String(digitalRead(waterPumpPin)));
  Serial1.println("output/pump1/state: "+ String(digitalRead(P_pump1Pin)));
  Serial1.println("output/pump2/state: "+ String(digitalRead(P_pump2Pin)));
  Serial1.println("output/pump3/state: "+ String(digitalRead(P_pump3Pin)));
  Serial1.println("output/pump4/state: "+ String(digitalRead(P_pump4Pin)));
  Serial1.println("output/EC_enable/state: "+ String(digitalRead(EC_enable)));

} 

void AddNutriant()
{
  digitalWrite(P_pump3Pin,HIGH);
  Serial.println("pump3: " + String(digitalRead(P_pump3Pin)));
  Serial1.println("output/pump3/state: " + String(digitalRead(P_pump3Pin))); //Send data to ESP32
  delay(3000);
  digitalWrite(P_pump3Pin,LOW);
  Serial.println("pump3: " + String(digitalRead(P_pump3Pin)));
  Serial1.println("output/pump3/state: " + String(digitalRead(P_pump3Pin))); //Send data to ESP32
}
void RaisePH()
{
  digitalWrite(P_pump2Pin,HIGH);
  Serial.println("pump2: " + String(digitalRead(P_pump2Pin)));
  Serial1.println("output/pump2/state: " + String(digitalRead(P_pump2Pin))); //Send data to ESP32
  delay(3000);
  digitalWrite(P_pump2Pin,LOW);
  Serial.println("pump2: " + String(digitalRead(P_pump2Pin)));
  Serial1.println("output/pump2/state: " + String(digitalRead(P_pump2Pin))); 
}
void LowerPH()
{
  digitalWrite(P_pump1Pin,HIGH);
  Serial.println("output/pump1: " + String(digitalRead(P_pump1Pin)));
  Serial1.println("output/pump1/state: " + String(digitalRead(P_pump1Pin))); //Send data to ESP32
  delay(3000);
  digitalWrite(P_pump1Pin,LOW);
  Serial.println("output/pump1: " + String(digitalRead(P_pump1Pin)));
}

void pulse()   //measure the quantity of square wave
{
  waterFlow += 1.0 / 450.0; // 450 pulses for 1 liter (see product parameters)
}

void setup() {
  Serial.begin(19200);
  Serial1.begin(9600);

  pinMode(lightPin, OUTPUT);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(P_pump1Pin, OUTPUT);

  pinMode(P_pump2Pin, OUTPUT);
  pinMode(P_pump3Pin, OUTPUT);
  pinMode(P_pump4Pin, OUTPUT);
  pinMode(EC_enable, OUTPUT);
  pinMode(WaterLevelPin, INPUT_PULLUP);
  pinMode(interruptPin, INPUT_PULLUP);

  //Report State
  prev_Mode = "Manual";
  Mode = "Manual";
  Serial1.println("state/Mode: " + Mode);

  //Report Parameters
  Serial1.println("parameter/SerialSendDelay: " + String(SerialSendDelay));
  Serial1.println("parameter/waterADJDelay: " + String(waterADJDelay));
  Serial1.println("parameter/range_ec_high: " + String(range_ec_high));
  Serial1.println("parameter/range_ec_low: " + String(range_ec_low));
  Serial1.println("parameter/range_ph_high: " + String(range_ph_high));
  Serial1.println("parameter/range_ph_low: " + String(range_ph_low));


  //Initilize Outputs Command States
  Serial1.println("output/Light/command: "+ String(digitalRead(lightPin)));
  Serial1.println("output/Water Pump/command: "+ String(digitalRead(waterPumpPin)));
  Serial1.println("output/pump1/command: "+ String(digitalRead(P_pump1Pin)));
  Serial1.println("output/pump2/command: "+ String(digitalRead(P_pump2Pin)));
  Serial1.println("output/pump3/command: "+ String(digitalRead(P_pump3Pin)));
  Serial1.println("output/pump4/command: "+ String(digitalRead(P_pump4Pin)));
  Serial1.println("output/Light/state: "+ String(digitalRead(lightPin)));
  Serial1.println("output/Water Pump/state: "+ String(digitalRead(waterPumpPin)));
  Serial1.println("output/pump1/state: "+ String(digitalRead(P_pump1Pin)));
  Serial1.println("output/pump2/state: "+ String(digitalRead(P_pump2Pin)));
  Serial1.println("output/pump3/state: "+ String(digitalRead(P_pump3Pin)));
  Serial1.println("output/pump4/state: "+ String(digitalRead(P_pump4Pin)));
  Serial1.println("output/EC_enable/state: "+ String(digitalRead(EC_enable)));


  ec.setCalibration(1.0572);//Replace the 1 with the calibrated K value if it's calibrated
  Serial.println("Default Calibration K=" + String(ec.getCalibration()));
  Serial1.println("parameter/EC_K: " + String(ec.getCalibration()));

  ph.begin();

  waterFlow = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), pulse, RISING);  //DIGITAL Pin 2: Interrupt 0
  
  
  phAVG.begin();
  ecAVG.begin();
  TempAVG.begin();



}



void loop() {


  Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);
  Conductivity = float(ecAVG.reading(Conductivity));

  if (digitalRead(EC_enable) == HIGH)
  {
    EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
    TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;

    Temp = ecpt.convVoltagetoTemperature_C((float)TE_Voltage/1000);
    Temp = Temp * 100;
    Temp = float(TempAVG.reading(Temp))/100;
    Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);
    Conductivity = float(ecAVG.reading(Conductivity));
  }

  if (digitalRead(EC_enable) == LOW)
  {
    voltagePH = analogRead(PH_PIN)/1024.0*5000;
    phValue = ph.readPH(voltagePH,Temp);
    phValue = phValue*100;
    phValue = float(phAVG.reading(phValue))/100;
    phValue = phValue + PHoffset;
  }
  
  POWERsensorValue = analogRead(POWERsensorPin);
  WaterLevelState = digitalRead(WaterLevelPin);

  DurationSinceECREad = millis() - timeOfRead;
  DurationSinceADJ = millis() - timeOfADJ;
  DurationSinceSerialSend = millis() - timeOfSerialSend;

  String Sensor_Readings = "Flow: " + String(waterFlow) +",PH: "+ phValue +",EC: "+ Conductivity +",TEMP: " + Temp + ",Power: "+ POWERsensorValue +",WaterLevel: "+ WaterLevelState +",Light: "+ digitalRead(lightPin) +",Water Pump: "+ digitalRead(waterPumpPin);


  if (Mode == "Manual"){prev_Mode = manualModeLoop(prev_Mode);}
  
  if (Mode == "Auto"){

    if (prev_Mode == "Manual")
    {
      digitalWrite(lightPin,HIGH);
      digitalWrite(waterPumpPin,HIGH);
      timeOfADJ = millis();
      prev_Mode = "Auto";
    }
    
    if(millis() > 300000 && DurationSinceADJ > waterADJDelay)
    {
      if(Conductivity < range_ec_low){ AddNutriant();timeOfADJ = millis();}
      else if (phValue < range_ph_low){ RaisePH();timeOfADJ = millis();}
      else if (phValue > range_ph_high){ LowerPH();timeOfADJ = millis();}
    }


    if(DurationSinceECREad > ECReadDelay)
    {
      
      timeOfRead = millis();
      int ec_state = digitalRead(EC_enable);
      
      if (ec_state == 1)
      {
        digitalWrite(EC_enable,LOW);
      }
      if (ec_state == 0)
      {
        digitalWrite(EC_enable,HIGH);
      }
      Serial1.println("output/EC_enable/state: "+ String(digitalRead(EC_enable)));
    }

  }
//report (flow, ph, ec, temp, power, water level)
  if(DurationSinceSerialSend > SerialSendDelay)
  {
    Serial.println(Sensor_Readings);
    Serial1.println("sensor/Flow: " + String(waterFlow)); //Send data to ESP32
    Serial1.println("sensor/PH: "+ String(phValue)); //Send data to ESP32
    Serial1.println("sensor/EC: "+ String(Conductivity));
    Serial1.println("sensor/EC_Input_Voltage: "+ String(EC_Voltage));
    Serial1.println("sensor/TEMP: " + String(Temp));
    Serial1.println("sensor/Power: "+ String(POWERsensorValue));
    Serial1.println("sensor/WaterLevel: "+ String(WaterLevelState));
    Serial1.println("output/Light/state: "+ String(digitalRead(lightPin)));
    Serial1.println("output/Water Pump/state: "+ String(digitalRead(waterPumpPin)));
    Serial1.println("output/pump1/state: "+ String(digitalRead(P_pump1Pin)));
    Serial1.println("output/pump2/state: "+ String(digitalRead(P_pump2Pin)));
    Serial1.println("output/pump3/state: "+ String(digitalRead(P_pump3Pin)));
    Serial1.println("output/pump4/state: "+ String(digitalRead(P_pump4Pin)));
    Serial1.println("output/EC_enable/state: "+ String(digitalRead(EC_enable)));
    

    timeOfSerialSend = millis();
  }



//code to recieve command from esp32
  while (Serial1.available())
  {
    c = Serial1.read(); //Read the incoming data and save to c variable, once character at a time

    if(c != '\n') { dataIn += c; } //If the End Data Identifier ('\n') not found, then append data c to the string variable dataIn
    else          { break;}        //If the end Data identifier ('\n') is found, break while loop  }
  }

  if(c=='\n')
  {

    int index = dataIn.indexOf(":");
    String Name = dataIn.substring(0, index);
    String Val = dataIn.substring(index);
    Val.replace(":","");
    Val.trim();

    Serial.println(Name);
    Serial.println(Val);

    if (Name == "Mode")
    {
      if (Val == "Auto"){Mode = "Auto";}
      if (Val == "Manual"){Mode = "Manual";}
    }
    if (Mode == "Manual")
    {
      if (Name == "LightCMD"){ SerialtoDigitalWrite(Val,lightPin);}
      //Serial1.println("sensor/LightCMD: "+ String(WaterLevelState));

      if (Name == "Water Pump CMD"){ SerialtoDigitalWrite(Val,waterPumpPin);}
      if (Name == "pump1CMD"){ SerialtoDigitalWrite(Val,P_pump1Pin);}
      if (Name == "pump2CMD"){ SerialtoDigitalWrite(Val,P_pump2Pin);}
      if (Name == "pump3CMD"){ SerialtoDigitalWrite(Val,P_pump3Pin);}
      if (Name == "pump4CMD"){ SerialtoDigitalWrite(Val,P_pump4Pin);}
      if (Name == "EC_enableCMD"){ SerialtoDigitalWrite(Val,EC_enable);}

    }
    c=0;
    dataIn="";
  }
}




#include <Arduino.h>
#include <SoftwareSerial.h>


#define LED_PIN 13
#define BLINK_DELAY 100

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

//Digital Inputs
int WaterLevelPin = 44;
int WaterLevelState = 0;

//Analog Inputs
//Flow Meter
int FLOWsensorPin = A0;
int FLOWsensorValue = 0;
//PH Meter
int pHsensorPin = A1;
int PHsensorValue = 0;
//Electrical Conductivity Meter
int ECsensorPin = A1;
int ECsensorValue = 0;
//Temperature Sensor
int TEMPsensorPin = A2;
int TEMPsensorValue = 0;
//Power Meter
int POWERsensorPin = A3;
int POWERsensorValue = 0;


//# Desired range for electrical conductivity
int range_ec_high = 1300;
int range_ec_low = 1000;
//# Desired range for pH
int range_ph_high = 6.2;
int range_ph_low = 5.5;
//# pH range that will immediately cause a pH warning
int range_ph_high_danger = 7.0;
int range_ph_low_danger = 5.0;

unsigned long startTime = millis();
unsigned long waterADJDelay = 300000;//300000;
unsigned long timeOfADJ = millis();
unsigned long DurationSinceADJ = 0;
unsigned long DurationSinceSerialSend = 0;
unsigned long SerialSendDelay = 1000;
unsigned long timeOfSerialSend = millis();
String Mode;
String prev_Mode;


void setup() {
  Serial.begin(19200);
  Serial1.begin(9600);

  pinMode(lightPin, OUTPUT);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(P_pump1Pin, OUTPUT);
  pinMode(P_pump2Pin, OUTPUT);
  pinMode(P_pump3Pin, OUTPUT);
  pinMode(P_pump4Pin, OUTPUT);
  pinMode(WaterLevelPin, INPUT);

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


}

void loop() {

  FLOWsensorValue = analogRead(FLOWsensorPin);
  PHsensorValue = analogRead(pHsensorPin);
  ECsensorValue = analogRead(ECsensorPin);
  TEMPsensorValue = analogRead(TEMPsensorPin);
  POWERsensorValue = analogRead(POWERsensorPin);
  WaterLevelState = digitalRead(WaterLevelPin);

  DurationSinceADJ = millis() - timeOfADJ;
  DurationSinceSerialSend = millis() - timeOfSerialSend;

  String Sensor_Readings = "Flow: " + String(FLOWsensorValue) +",PH: "+ PHsensorValue +",EC: "+ ECsensorValue +",TEMP: " + TEMPsensorValue + ",Power: "+ POWERsensorValue +",WaterLevel: "+ WaterLevelState +",Light: "+ digitalRead(lightPin) +",Water Pump: "+ digitalRead(waterPumpPin);

 


/* 
if(PHsensorValue == 0 or ECsensorValue == 0){

    if(PHsensorValue == 0){
      //TODO;
    }
      
    if(ECsensorValue == 0){
      //TODO;
    }
}
 */
  if (Mode == "Manual")
  {
    if (prev_Mode == "Auto")
    {
      prev_Mode = "Manual";
    }
    
    /* code */
  }
  
  

  //water adjustment code
  if(Mode == "Auto"){
    if (prev_Mode == "Manual")
    {
      digitalWrite(lightPin,HIGH);
      digitalWrite(waterPumpPin,HIGH);
      timeOfADJ = millis();
      prev_Mode = "Auto";
    }
    

  //if( startTime > 300000 && DurationSinceADJ > waterADJDelay){
    if(millis() > 3000 && DurationSinceADJ > waterADJDelay){

      if(ECsensorValue < range_ec_low){ // add nutriants
        digitalWrite(P_pump3Pin,HIGH);
        Serial.println("pump3: " + String(digitalRead(P_pump3Pin)));
        Serial1.println("output/pump3/state: " + String(digitalRead(P_pump3Pin))); //Send data to ESP32
        delay(3000);
        digitalWrite(P_pump3Pin,LOW);
        Serial.println("pump3: " + String(digitalRead(P_pump3Pin)));
        Serial1.println("output/pump3/state: " + String(digitalRead(P_pump3Pin))); //Send data to ESP32
        timeOfADJ = millis();
      }
      // else(ECsensorValue > range_ec_high){ 
      //   //TODO
        
      // }
      else if (PHsensorValue < range_ph_low){ // add base
        digitalWrite(P_pump2Pin,HIGH);
        Serial.println("pump2: " + String(digitalRead(P_pump2Pin)));
        Serial1.println("output/pump2/state: " + String(digitalRead(P_pump2Pin))); //Send data to ESP32
        delay(3000);
        digitalWrite(P_pump2Pin,LOW);
        Serial.println("pump2: " + String(digitalRead(P_pump2Pin)));
        Serial1.println("output/pump2/state: " + String(digitalRead(P_pump2Pin))); 
        timeOfADJ = millis();
      }
      else if (PHsensorValue > range_ph_high){ // add acid
        digitalWrite(P_pump1Pin,HIGH);
        Serial.println("output/pump1: " + String(digitalRead(P_pump1Pin)));
        Serial1.println("output/pump1/state: " + String(digitalRead(P_pump1Pin))); //Send data to ESP32
        delay(3000);
        digitalWrite(P_pump1Pin,LOW);
        Serial.println("output/pump1: " + String(digitalRead(P_pump1Pin)));
        Serial1.println("output/pump1/state: " + String(digitalRead(P_pump1Pin))); //Send data to ESP32
        timeOfADJ = millis();
      }
    }
  }


//report (flow, ph, ec, temp, power, water level)
  if(DurationSinceSerialSend > SerialSendDelay)
  {



    Serial.println(Sensor_Readings);
    Serial1.println("sensor/Flow: " + String(FLOWsensorValue)); //Send data to ESP32
    Serial1.println("sensor/PH: "+ String(PHsensorValue)); //Send data to ESP32
    Serial1.println("sensor/EC: "+ String(ECsensorValue));
    Serial1.println("sensor/TEMP: " + String(TEMPsensorValue));
    Serial1.println("sensor/Power: "+ String(POWERsensorValue));
    Serial1.println("sensor/WaterLevel: "+ String(WaterLevelState));
    Serial1.println("output/Light/state: "+ String(digitalRead(lightPin)));
    Serial1.println("output/Water Pump/state: "+ String(digitalRead(waterPumpPin)));
    Serial1.println("output/pump1/state: "+ String(digitalRead(P_pump1Pin)));
    Serial1.println("output/pump2/state: "+ String(digitalRead(P_pump2Pin)));
    Serial1.println("output/pump3/state: "+ String(digitalRead(P_pump3Pin)));
    Serial1.println("output/pump4/state: "+ String(digitalRead(P_pump4Pin)));

    
    


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
      if (Name == "LightCMD")
      {
        if (Val == "1"){digitalWrite(lightPin,HIGH);}
        if (Val == "0"){digitalWrite(lightPin,LOW);}
        Serial1.println("sensor/LightCMD: "+ String(WaterLevelState));
      }

      if (Name == "Water Pump CMD")
      {
        if (Val == "1"){digitalWrite(waterPumpPin,HIGH);}
        if (Val == "0"){digitalWrite(waterPumpPin,LOW);}
      }

      if (Name == "pump1CMD")
      {
        if (Val == "1"){digitalWrite(P_pump1Pin,HIGH);}
        if (Val == "0"){digitalWrite(P_pump1Pin,LOW);}
      }

      if (Name == "pump2CMD")
      {
        if (Val == "1"){digitalWrite(P_pump2Pin,HIGH);}
        if (Val == "0"){digitalWrite(P_pump2Pin,LOW);}
      }

      if (Name == "pump3CMD")
      {
        if (Val == "1"){digitalWrite(P_pump3Pin,HIGH);}
        if (Val == "0"){digitalWrite(P_pump3Pin,LOW);}
      }

      if (Name == "pump4CMD")
      {
        if (Val == "1"){digitalWrite(P_pump4Pin,HIGH);}
        if (Val == "0"){digitalWrite(P_pump4Pin,LOW);}
      } 



    }
    c=0;
    dataIn="";
  }
}
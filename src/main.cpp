#include <Arduino.h>
#include <SoftwareSerial.h>


#define LED_PIN 13
#define BLINK_DELAY 100

int n = 0;

//Analog Inputs
int pHsensorPin = A0;
int PHsensorValue = 0;
//Digital Outputs
int lightPin = 22;
int waterPumpPin = 23;
int P_pump1Pin = 24;
int P_pump2Pin = 25;
int P_pump3Pin = 26;
int P_pump4Pin = 27;
//Digital Inputs
int WaterLevelPin = 28;
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
unsigned long waterADJDelay = 300000;
unsigned long timeOfADJ = millis();
unsigned long DurationSinceADJ = 0;

void setup() {
  Serial.begin(9600);

  pinMode(lightPin, OUTPUT);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(P_pump1Pin, OUTPUT);
  pinMode(P_pump2Pin, OUTPUT);
  pinMode(P_pump3Pin, OUTPUT);
  pinMode(P_pump4Pin, OUTPUT);
  pinMode(P_pump4Pin, INPUT);


}

void loop() {

  FLOWsensorValue = analogRead(FLOWsensorPin);
  PHsensorValue = analogRead(pHsensorPin);
  ECsensorValue = analogRead(ECsensorPin);
  TEMPsensorValue = analogRead(TEMPsensorPin);
  POWERsensorValue = analogRead(POWERsensorPin);
  WaterLevelState = digitalRead(WaterLevelPin);

  DurationSinceADJ = millis() - timeOfADJ;

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


//water adjustment code
if( startTime > 300000 && DurationSinceADJ > waterADJDelay){


  if(ECsensorValue < range_ec_low){ // add nutriants
    digitalWrite(P_pump3Pin,HIGH);
    delay(3000);
    digitalWrite(P_pump3Pin,LOW);
    timeOfADJ = millis();
  }
  // else(ECsensorValue > range_ec_high){ 
  //   //TODO
    
  // }
  else if (PHsensorValue < range_ph_low){ // add base
    digitalWrite(P_pump2Pin,HIGH);
    delay(3000);
    digitalWrite(P_pump2Pin,LOW);
    timeOfADJ = millis();
  }
  else if (PHsensorValue > range_ph_high){ // add acid
    digitalWrite(P_pump1Pin,HIGH);
    delay(3000);
    digitalWrite(P_pump1Pin,LOW);
    timeOfADJ = millis();
  }
}

//

//code to control lights
//todo
//code to control pump
//todo
//code to communicate with esp32
//report (flow, ph, ec, temp, power, water level)



}
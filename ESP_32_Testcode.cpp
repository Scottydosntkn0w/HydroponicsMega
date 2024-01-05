#define RXp2 16
#define TXp2 17

char c;
String dataIn;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
}
void loop() {

  Serial2.print("ESP32 Says: Hello Mega, How Are you? \n"); //Send data to Uno
  delay(200);

  while(Serial2.available() ) //Receive data from Uno
  {
    c = Serial2.read(); //Read the incoming data and save to c variable, once character at a time

    if(c != '\n') { dataIn += c; } //If the End Data Identifier ('\n') not found, then append data c to the string variable dataIn
    else          { break;}        //If the end Data identifier ('\n') is found, break while loop
  }

  if(c=='\n')
  {
    Serial.println(dataIn);

    c=0;
    dataIn="";
  }

}


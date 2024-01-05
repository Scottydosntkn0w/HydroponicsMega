

char c;
String dataIn;

void setup() {
  Serial.begin(19200);
  Serial1.begin(9600);
}
void loop() {



  while (Serial1.available())
  {
    c = Serial1.read(); //Read the incoming data and save to c variable, once character at a time

    if(c != '\n') { dataIn += c; } //If the End Data Identifier ('\n') not found, then append data c to the string variable dataIn
    else          { break;}        //If the end Data identifier ('\n') is found, break while loop 
  }

    if(c=='\n')
  {
    Serial.println(dataIn);
    Serial1.print("Mega Says: Hello ESP32, How Are you? \n"); //Send data to ESP32


    c=0;
    dataIn="";
  }


}
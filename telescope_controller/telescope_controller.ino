#include <SPI.h>

const int bufferSize = 255;
const unsigned long baudrate = 9600;
const SPISettings settings = SPISettings(baudrate, MSBFIRST, SPI_MODE0);

void setup() {
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);
    
  digitalWrite(SS, HIGH); // LOW on communication through SPI
  
  Serial.begin(baudrate);
  Serial.println(MOSI);
  Serial.println(MISO);
  Serial.println(SCK);
  Serial.println(SS);
}

void loop() {
  char buf[bufferSize] = "";
  unsigned long bufferLen = 0;

  delay(50);
  
  while (Serial.available() && bufferLen < bufferSize) {
    int inByte = Serial.read();
    Serial.println(inByte);
    Serial.println(char(inByte));
    buf[bufferLen] = char(inByte);
    bufferLen++;
  }

  if (bufferLen > 0) {
    Serial.print("Writing to SPI:");
    Serial.println(buf);
    
    SPI.beginTransaction(settings);
    digitalWrite(SS, LOW);
    SPI.transfer(buf, bufferLen);
    /*
    for (int i = 0; i < bufferLen; i++) {
      Serial.println(0 + buf[i]);'
    }
    */
    digitalWrite(SS, HIGH);
    SPI.endTransaction();
  }
}

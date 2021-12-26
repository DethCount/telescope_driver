#include <SPI.h>

const int BUFFER_SIZE = 1024;
const unsigned long baudrate = 9600;
const SPISettings settings = SPISettings(baudrate, MSBFIRST, SPI_MODE0);

void spiWrite(char buf[BUFFER_SIZE], int bufferLen) {
  SPI.beginTransaction(settings);
  digitalWrite(SS, LOW);
  SPI.transfer(buf, bufferLen);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
}

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

  char stopMsg[5] = {'S', 'T', 'O', 'P', '\n'};
  spiWrite(stopMsg, 5);
}

void loop() {
  char buf[BUFFER_SIZE] = "";
  int bufferLen = 0;
  
  while (Serial.available()) {
    bufferLen = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
  }

  if (bufferLen > 0) {
    if (bufferLen < BUFFER_SIZE) {
      buf[bufferLen] = '\n';
      bufferLen++;
    }
    
    Serial.print("Write to SPI:");
    Serial.println(buf);
    spiWrite(buf, bufferLen);
  }
}

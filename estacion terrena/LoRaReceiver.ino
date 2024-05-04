#include <SPI.h>
#include <LoRa.h>

#define MOSI 27
#define MISO 19
#define RST 14
#define CS 18
#define DIO0 26
#define SCLK 5

const int frequency = 915E6;

void setup() {
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

    LoRa.setSpreadingFactor(9);
    LoRa.setCodingRate4(0);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setPreambleLength(12);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    //Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    //Serial.print("' with RSSI ");
    Serial.println();
  }
}

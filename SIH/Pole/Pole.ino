#include <SPI.h>
#include <LoRa.h>

void transmitter(const String& data) {
  unsigned long startTime = millis(), transmissionDurationMillis = 10000;

  while (millis() - startTime < transmissionDurationMillis) {
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
  }
}

String receiver() {
  unsigned long receiverTimeout = 7000; // Set your desired timeout in milliseconds
  String str = "";
  unsigned long startTime = millis();

  while (millis() - startTime < receiverTimeout) {
    int packetSize = LoRa.parsePacket();

    if (packetSize) {
      Serial.print("Received packet '");

      while (LoRa.available()) {
        char c = LoRa.read();
        Serial.print(c);
        str += c;
      }

      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
      return str;
    }
  }

  Serial.println("Receiver timeout");
  return "";
}

void pole(const String& time, const String& dir) {
  Serial.println("Pole is set to transmit ");
  unsigned long startTime = millis(), transmissionDuration = 10000;
  String ack = receiver(); // Receive an acknowledgment

  if (ack == "V10") {
    Serial.println("Verified ");
    while (millis() - startTime < transmissionDuration) {
      // Transmit time
      transmitter(time);
      delay(5000); // Delay between time and direction transmission
      // Transmit direction
      transmitter(dir);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("LoRa Sender");
  LoRa.begin(433E6);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  String t = "T10", d = "D0";

  pole(t, d);
  delay(5000);

  d = "D1";
  pole(t, d);
  delay(5000);
  d = "D2";
  pole(t, d);
  delay(5000);
  d = "D3";
  pole(t, d);
  delay(5000);
  d = "D1";
  pole(t, d);
  delay(5000);
}

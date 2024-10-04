#include <SPI.h>
#include <LoRa.h>

#define SS_PIN 5
#define RST_PIN 14
#define DIO0_PIN 2
#define LORA_FREQUENCY 433E6
#define SYNC_WORD 0xF3
#define RECEIVER_TIMEOUT 5000

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!initializeLoRa()) {
    Serial.println("LoRa initialization failed. Check connections and restart.");
    while (1);  // Loop indefinitely on error
  }
}

void loop() {
  String msg = receiver();
  Serial.println(msg);
}

bool initializeLoRa() {
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error initializing LoRa module.");
    return false;
  }

  LoRa.setSyncWord(SYNC_WORD);
  return true;
}

String receiver() {
  String str = "";
  unsigned long startTime = millis();

  while (millis() - startTime < RECEIVER_TIMEOUT) {
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
  return "-1";  // Return an empty string on timeout
}

#include <SPI.h>
#include <LoRa.h>
int counter = 0;
String base64decode(String encoded) {
  const char* encoding = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  int val = 0, valb = -6;
  String decoded = "";

  for (char c : encoded) {
    if (c == '=') {
      break;
    }

    val = (val << 6) + (strchr(encoding, c) - encoding);
    valb += 6;

    if (valb >= 0) {
      decoded += char((val >> valb) & 0xFF);
      valb -= 8;
    }
  }

  return decoded;
}
void transmitter(String message)
{
  Serial.println("Transmitting: ");
  Serial.print(message);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}
unsigned long receiverTimeout = 5000; // Set your desired timeout in milliseconds

String receiver() {
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

void setup() 
{
  Serial.begin(115200);
  Serial.println("LoRa Begin");
  LoRa.begin(433E6);
  if (!LoRa.begin(433E6)) 
  {
    Serial.println("Starting LoRa failed!");
    while(1);
  }
}
void loop() 
{
  String msg = "Hi kumar "+String(counter);
  transmitter(msg);
  delay(5000);
  String message = receiver();
  Serial.println(message);
  
  // Uncomment Line 21 and 22  and comment line 19 and 18 for temperature in Farenhite only
  //float tempf = (tempc*9.0)/5.0 + 32;
  //String msg = "Temperature is " + String(tempf) + "°F";

  //Uncomment Line 25 and 26 for temperature in both Celcius and Farenhite
  //float tempf = (tempc*9.0)/5.0 + 32;
  // msg  = msg + "/"+ String(tempf) + "°F"
   
  //Serial.print("Sending message ");
  //Serial.println(msg);
  
  //LoRa.beginPacket();
  //LoRa.print(msg);
  //LoRa.endPacket();
  counter++;
  
  delay(5000);
}
#include <SPI.h>
#include <LoRa.h>
void setup() 
{
  Serial.begin(115200);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(433E6)) 
  {
    Serial.println("Starting LoRa failed!");
    while(1);
  }
}
void loop() 
{
  String msg = "/P0";
  
  // Uncomment Line 21 and 22  and comment line 19 and 18 for temperature in Farenhite only
  //float tempf = (tempc*9.0)/5.0 + 32;
  //String msg = "Temperature is " + String(tempf) + "°F";

  //Uncomment Line 25 and 26 for temperature in both Celcius and Farenhite
  //float tempf = (tempc*9.0)/5.0 + 32;
  // msg  = msg + "/"+ String(tempf) + "°F"
   
  Serial.print("Sending message ");
  Serial.println(msg);
  
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  delay(1000);
}
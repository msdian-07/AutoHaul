#include <SPI.h>
#include <LoRa.h>

#define ENR 32 // Enables motors Right
#define ENL 33 // Enables motors Left
#define IN1 25 // L298N in1 motors Right
#define IN2 26 // L298N in2 motors Right
#define IN3 27 // L298N in3 motors Left
#define IN4 14 // L298N in4 motors Left

enum Direction { STOP, FORWARD, REVERSE, LEFT, RIGHT };
int Speed = 450; // initial speed 400-1024
int SpeedDiv = 3;
int counter = 0;

String time = ""; // Declare time variable
String dir = "";  // Declare dir variable

unsigned long motorStartTime = 0; // Variable to store the start time of motor action
unsigned long motorDuration = 0; // Variable to store the duration of motor action

void Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENR, Speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENL, Speed);
}

void Reverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENR, Speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENL, Speed);
}

void Right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENR, Speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENL, Speed);
}

void Left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENR, Speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENL, Speed);
}

void StopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void transmitter(const uint8_t* data, size_t dataSize) {
  unsigned long startTime = millis(), transmissionDurationMillis = 10000;

  while (millis() - startTime < transmissionDurationMillis) {
    LoRa.beginPacket();
    LoRa.write(data, dataSize);
    LoRa.endPacket();
  }
}

String receiver() {
  unsigned long receiverTimeout = 5000;
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

void vehicle(String vid) {
  unsigned long startTime = millis(), transmissionDuration = 10000;
  transmitter(vid.c_str(), vid.length());
  String a = receiver();
  delay(5000);
  String b = receiver();
  time = (a[0] == 'T') ? a : b;
  dir = (a[0] == 'D') ? a : b;
  if (a[0]=="T"){ Serial.println("a is t");}
  Direction commandDir = static_cast<Direction>(dir.substring(1).toInt());

  switch (commandDir) {
    case FORWARD:
      Serial.println("Forward");
      Forward();
      motorStartTime = millis();
      motorDuration = time.toInt() * 1000; // Convert time to milliseconds
      while (millis() - motorStartTime < motorDuration) {
        // Keep the motors running for the specified time
      }
      StopMotors(); // Stop the motors after the specified time
      break;
    case REVERSE:
      Serial.println("Reverse");
      Reverse();
      motorStartTime = millis();
      motorDuration = time.toInt() * 1000;
      while (millis() - motorStartTime < motorDuration) {
        // Keep the motors running for the specified time
      }
      StopMotors();
      break;
    case LEFT:
      Serial.println("Left");
      Left();
      motorStartTime = millis();
      motorDuration = time.toInt() * 1000;
      while (millis() - motorStartTime < motorDuration) {
        // Keep the motors running for the specified time
      }
      StopMotors();
      break;
    case RIGHT:
      Serial.println("Right");
      Right();
      motorStartTime = millis();
      motorDuration = time.toInt() * 1000;
      while (millis() - motorStartTime < motorDuration) {
        // Keep the motors running for the specified time
      }
      StopMotors();
      break;
    case STOP:
      Serial.println("Stop");
      StopMotors();
      break;
    default:
      // Handle unexpected direction
      Serial.println("Unknown direction");
      break;
Serial.println("Function ends");
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
  int t = 10, d = 0;

  vehicle("V10");
  delay(5000);
}

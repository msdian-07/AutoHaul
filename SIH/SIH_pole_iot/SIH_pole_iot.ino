#include <FirebaseESP32.h>
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#define ENR 32 // Enables motors Right
#define ENL 33 // Enables motors Left
#define IN1 25 // L298N in1 motors Right
#define IN2 26 // L298N in2 motors Right
#define IN3 27 // L298N in3 motors Left
#define IN4 12 // L298N in4 motors Left
#define SS_PIN 5
#define RST_PIN 14
#define DIO0_PIN 2
#define LORA_FREQUENCY 433E6
#define SYNC_WORD 0xF3
#define RECEIVER_TIMEOUT 7000

#define WIFI_SSID "Abcdefg" 
#define WIFI_PASSWORD "12349876" 
#define FIREBASE_HOST "********************************"
#define FIREBASE_Authorization_key "********************************8"
String FBP="**************************88";
int Speed = 600; // initial speed 400-1024
int SpeedDiv = 3;
int Ttime,dir;
FirebaseData firebaseData;
FirebaseJson json;

FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;


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


void setup() {

 Serial.begin(115200);
   WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
   Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("LoRa Receiver");

  if (!initializeLoRa()) {
    Serial.println("LoRa initialization failed. Check connections and restart.");
    while (1);  // Loop indefinitely on error
  }
  Serial.println();
  pinMode(ENR, OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.api_key = FIREBASE_Authorization_key; 
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_Authorization_key; // Set legacy token
  Firebase.begin(&firebaseConfig, &firebaseAuth);

}

void loop() {
    unsigned long startTime = millis(), transmissionDurationMillis;
    String path= receiver();
    Serial.println(path[1]);
    Serial.println(path.charAt(1) == 'P');
    
    if (path.charAt(1) == 'P'){
        String timePath = path +"/time";
        String dirPath = path +"/dir";
        Serial.println(timePath);
       Serial.println("Reading Firebase data...");
if (Firebase.getString(firebaseData, timePath)) {
    Ttime = firebaseData.stringData().toInt();
    Serial.print("Time: ");
    Serial.println(Ttime);
} else {
    Serial.println("Failed to read time from Firebase");
}

if (Firebase.getString(firebaseData, dirPath)) {
    dir = firebaseData.stringData().toInt();
    Serial.print("Direction: ");
    Serial.println(dir);
} else {
    Serial.println("Failed to read direction from Firebase");
}
        Serial.println(Ttime);
        Serial.println(dir);
            unsigned long transmissionDurationMillis = 1000UL * Ttime; // Ensure that the constant is of type unsigned long

            unsigned long startTime = millis();
            
            while (millis() - startTime < transmissionDurationMillis) {
                    // Execute motor control based on the received direction
                    if (dir == 1) {
                        Forward();
                        Serial.println("FORWARD");
                    } else if (dir == 2) {
                        Reverse();
                        Serial.println("REVERSE");
                    } else if (dir == 3) {
                        Left();
                        Serial.println("LEFT");
                    } else if (dir == 4) {
                        Right();
                        Serial.println("RIGHT");
                    } else if (dir == 0) {
                        StopMotors();
                        Serial.println("STOP");
                    }
                }
            }
  else{
    Serial.println("Not Near Pole ");
   StopMotors();
  }
  ESP.restart();
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



bool initializeLoRa() {
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error initializing LoRa module.");
    return false;
  }

  LoRa.setSyncWord(SYNC_WORD);
  return true;
}

#include <FirebaseESP32.h>
#include <WiFi.h>
#define ENR 32 // Enables motors Right
#define ENL 33 // Enables motors Left
#define IN1 25 // L298N in1 motors Right
#define IN2 26 // L298N in2 motors Right
#define IN3 27 // L298N in3 motors Left
#define IN4 14 // L298N in4 motors Left
#define TRIG_PIN 16 // Trigger pin for ultrasonic sensor
#define ECHO_PIN 17 // Echo pin for ultrasonic sensor
#define OBSTACLE_DISTANCE_THRESHOLD 20 
#define WIFI_SSID "Abcdefg" 
#define WIFI_PASSWORD "12349876" 
#define FIREBASE_HOST "https://sih1336-default-rtdb.firebaseio.com/"
#define FIREBASE_Authorization_key "AIzaSyDJW5GQS6PXb-S1PqTsqnA2c20xpjGD0Iw"
String FBP="https://sih1336-default-rtdb.firebaseio.com";
int Speed = 800; // initial speed 400-1024
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
  Serial.println();
  pinMode(ENR, OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, OUTPUT);

  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.api_key = FIREBASE_Authorization_key; 
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_Authorization_key; // Set legacy token
  Firebase.begin(&firebaseConfig, &firebaseAuth);

}

void loop() {
    String paths[] = {"/P0", "/P1", "/P2", "/P3"};
    unsigned long startTime = millis(), transmissionDurationMillis;
    for (String path : paths) {
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
    }
    

bool isObstacleDetected() {
    // Trigger ultrasonic sensor to measure distance
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read the duration of the pulse on the echo pin
    long duration = pulseIn(ECHO_PIN, HIGH);

    // Convert the duration to distance in centimeters
    float distance = duration * 0.034 / 2;

    // Check if obstacle is within the threshold distance
    return (distance < OBSTACLE_DISTANCE_THRESHOLD);
}

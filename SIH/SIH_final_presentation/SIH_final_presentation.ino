#include <FirebaseESP32.h>
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define RXD2 16
#define TXD2 17
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
#define FIREBASE_HOST "********************"
#define FIREBASE_Authorization_key "*******************************"
String FBP="************************************";
int Speed = 600; // initial speed 400-1024
int SpeedDiv = 3;
int Ttime,dir;
FirebaseData firebaseData;
FirebaseJson json;

FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;
int dist; /*----actual distance measurements of LiDAR---*/
int strength; /*----signal strength of LiDAR----------------*/
float temprature;
unsigned char check;        /*----save check value------------------------*/
int i;
unsigned char uart[9];  /*----save data measured by LiDAR-------------*/
const int HEADER=0x59; /*----frame header of data package------------*/
int rec_debug_state = 0x01;//receive state for frame
Adafruit_MPU6050 mpu;
void setup() {

 Serial.begin(115200);
   while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
/*
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;}*/
 Serial.println("\nBenewake TFmini-S UART LiDAR Program");
  Serial2.begin(115200);
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

int distance(){
  if (Serial2.available()) // check if serial port has data input
  {
    if (rec_debug_state == 0x01)
    {  //the first byte
      uart[0] = Serial2.read();
      if (uart[0] == 0x59)
      {
        check = uart[0];
        rec_debug_state = 0x02;
      }
    }
    else if (rec_debug_state == 0x02)
    {//the second byte
      uart[1] = Serial2.read();
      if (uart[1] == 0x59)
      {
        check += uart[1];
        rec_debug_state = 0x03;
      }
      else
      {
        rec_debug_state = 0x01;
      }
    }
    else if (rec_debug_state == 0x03)
    {
      uart[2] = Serial2.read();
      check += uart[2];
      rec_debug_state = 0x04;
    }
    else if (rec_debug_state == 0x04)
    {
      uart[3] = Serial2.read();
      check += uart[3];
      rec_debug_state = 0x05;
    }
    else if (rec_debug_state == 0x05)
    {
      uart[4] = Serial2.read();
      check += uart[4];
      rec_debug_state = 0x06;
    }
    else if (rec_debug_state == 0x06)
    {
      uart[5] = Serial2.read();
      check += uart[5];
      rec_debug_state = 0x07;
    }
    else if (rec_debug_state == 0x07)
    {
      uart[6] = Serial2.read();
      check += uart[6];
      rec_debug_state = 0x08;
    }
    else if (rec_debug_state == 0x08)
    {
      uart[7] = Serial2.read();
      check += uart[7];
      rec_debug_state = 0x09;
    }
    else if (rec_debug_state == 0x09)
    {
      uart[8] = Serial2.read();
      if (uart[8] == check)
      {
        dist = uart[2] + uart[3] * 256;//the distance
        strength = uart[4] + uart[5] * 256;//the strength
        temprature = uart[6] + uart[7] * 256;//calculate chip temprature
        temprature = temprature / 8 - 256;
        Serial.print("dist = ");
        Serial.print(dist); //output measure distance value of LiDAR
        Serial.print('\n');
        Serial.print("strength = ");
        Serial.print(strength); //output signal strength value
        Serial.print('\n');
        Serial.print("\t Chip Temprature = ");
        Serial.print(temprature);
        Serial.println(" celcius degree"); //output chip temperature of Lidar
        while (Serial2.available())
        {
          Serial2.read();
        } // This part is added because some previous packets are there in the buffer so to clear serial buffer and get fresh data.
        delay(100);
      }
      rec_debug_state = 0x01;
    }
  }
  return dist;
}
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
                    if (dir == 1 && distance()>20) {
                        Forward();
                        Serial.println("FORWARD");
                        Serial.println(speed());
                    } else if (dir == 2 && distance()>20) {
                        Reverse();
                        Serial.println("REVERSE");
                        Serial.println(speed());
                    } else if (dir == 3 && distance()>20) {
                        Left();
                        Serial.println("LEFT");
                        Serial.println(speed());
                    } else if (dir == 4 && distance()>20) {
                        Right();
                        Serial.println("RIGHT");
                        Serial.println(speed());
                    } else if (dir == 0 && distance()>20) {
                        StopMotors();
                        Serial.println("STOP");
                        Serial.println(speed());
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
float speed() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float speed = sqrt(a.acceleration.x * a.acceleration.x +
                     a.acceleration.y * a.acceleration.y +
                     a.acceleration.z * a.acceleration.z);
  return speed;
}

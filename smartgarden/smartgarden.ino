#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#define moisturePin 34
#define pompe 26
#include <Arduino.h>

// Broches des LEDs
const int ledPins[] = {12, 14, 36,22,23,21,13};
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// Broche du capteur de mouvement
const int motionSensorPin = 15;

// Durée d'allumage des LEDs en millisecondes
const unsigned long ledDuration = 5000; 

// Variables de gestion du mouvement
bool isMotionDetected = false;
unsigned long lastMotionTime = 0;


const char* ssid = "Galaxy A52A1A4";
const char* password = "sirine1234";
WebServer server(80);

int trigPin1 = 27;
int echoPin1 = 25;
int trigPin2 = 33;
int echoPin2 = 32;
int servoPin = 18;

int maxDistance =11 ;
Servo myservo;
int moistureThreshold = 50; // moisture level threshold for watering

//int sensorValue = analogRead(34);  
//int humidity = map(sensorValue,1040, 4095, 100, 0);
void setup() {
  Serial.begin(115200);
   // Configuration des broches des LEDs en sortie
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Configuration de la broche du capteur de mouvement en entrée
  pinMode(motionSensorPin, INPUT);

  // Initialisation du dernier temps de mouvement
  lastMotionTime = millis();
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);
  
  pinMode(moisturePin, INPUT);
  pinMode(pompe, OUTPUT);
  //digitalWrite(RELAY_PIN, LOW);//pompe off 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

 
  server.on("/", []() {
   int distance2 = getDistance(trigPin2, echoPin2);
  int percent = map(distance2, 0, maxDistance, 100, 0);
  String gauge = "<div style='background-color: #e6e6e6; width: 100px; height: 20px; text-align:center;'><div style='background-color: #4CAF50; width: " 
  + String(percent) + "%; height: 20px; margin-top:10px;'></div><p style='margin-top:5px;'>Level Bin : " + String(percent) + "%</p></div>";

// convert the sensor value to a percentage

int sensorValue = analogRead(34);  
 int humidity = map(sensorValue ,1040, 4095, 100, 0);

String moisture_gauge = "<div style='background-color: #FFFF00; width: 50px; height: 20px; text-align:center;'><div style='background-color: #4CAF50; width: " + String(humidity) + "%; height: 20px; margin-top:100px;'></div><p style='margin-top:10px;'>humidity: " + String(humidity) + "%</p></div>";

server.send(200, "text/html", gauge + moisture_gauge);

  });

  server.begin();
}

void loop() {
   // Vérifier si un mouvement est détecté
  if (digitalRead(motionSensorPin) == HIGH) {
    isMotionDetected = true;
    lastMotionTime = millis();
  }

  // Allumer les LEDs si un mouvement est détecté et respecter la durée d'allumage
  if (isMotionDetected && (millis() - lastMotionTime) < ledDuration) {
    for (int i = 0; i < numLeds; i++) {
      digitalWrite(ledPins[i], HIGH);
    }
  } else {
    // Éteindre les LEDs si la durée d'allumage est écoulée ou si aucun mouvement n'est détecté
    for (int i = 0; i < numLeds; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    isMotionDetected = false;
  }
  int distance1 = getDistance(trigPin1, echoPin1);
  if (distance1 <= 10) {
    myservo.write(90);
    delay(5000);
    myservo.write(0);}
       //read the humidity from the sensor
  // convert the sensor value to a percentage
   // humidity = map(humidity, 0, 4095, 0, 100);
   //print the humidity to the serial port
 // Serial.print("Humidity: ");
 // Serial.print(humidity);
  //Serial.println("%");

  int sensorValue = analogRead(34);  // Read the analog value from sensor
  int humidity = map(sensorValue,1040, 4095, 100, 0);// map the 10-bit data to 8-bit data_
  Serial.println("humidity: ");
  Serial.print(humidity);

  delay(500); 

  // check if the humidity is below the threshold
  if (humidity <= moistureThreshold) {
  digitalWrite(pompe, HIGH);  } // activer la pompe
 // Serial.println("pump on");
 // delay(20000); }// attendre la durée de l'activation
  else {
  digitalWrite(pompe, LOW); // désactiver la pompe
   //Serial.println("pump off");
  //delay(3000); // attendre une seconde avant de recommencer
  }
  delay(1000);

//Serial.println("pause");
//delay(600000); // pause 10mns le temps que l'eau se diffuse autour du capteur
//delay(10000); // pause
 
server.handleClient();}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}
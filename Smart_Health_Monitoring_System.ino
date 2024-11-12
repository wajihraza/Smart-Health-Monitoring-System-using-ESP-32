#define BLYNK_TEMPLATE_ID "TMPL6UjgPcu9S"
#define BLYNK_TEMPLATE_NAME "ESD Semester Project"
#define BLYNK_AUTH_TOKEN "eS0FhRQ4kxeMJ_8daEYIAs7tLG7l6tiJ"

#define TRIG_PIN 26 
#define ECHO_PIN 25 
#define BUZZER_PIN 18
#define DISTANCE_THRESHOLD 5 // centimeters
#define HEARTBEAT_SENSOR_PIN 33
#define LED21 32 //heartbeat sensor
#define HEARTBEAT_THRESHOLD 3000

#define LED32 2 //gas sensor do pin
#define GAS_SENSOR_INPUT 4

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPSPlus.h>

BlynkTimer timer;

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Lenovo-T490";
char pass[] = "wajih001";



TinyGPSPlus gps;
void gps_sensor_task(void *pvParameter) {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  //delay(3000);
  while(1) 
  {
    while (Serial2.available() > 0) 
    {
    char c = Serial2.read();
    gps.encode(c); // Feed the character to the TinyGPS++ object
    }

  // Check if there is new GPS data available
    if (gps.location.isUpdated()) 
    {
      // Display latitude, longitude, and speed
      Serial.print("\nLocation: ");
      Serial.print(gps.location.lat(), 6);
      float latitude = gps.location.lat();
      Blynk.virtualWrite(V4, latitude);
      Serial.print(", ");
      Serial.print(gps.location.lng(), 6);
      Serial.print("\nSpeed: ");
      Serial.print(gps.speed.kmph());
      Serial.println("km/h");
    }

    // Print additional information for debugging
    if (gps.altitude.isUpdated()) 
    {
      Serial.print("Altitude: ");
      float altitude = gps.altitude.meters();
      Serial.print(gps.altitude.meters());
      Serial.println("meters");
          // Check if there is date data available
    if (gps.date.isValid()) {
      Serial.print("\nDate: ");
      Serial.print(gps.date.year());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
    }
      Blynk.virtualWrite(V3, altitude);
      
    }
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}


int sensor_Aout;
void gas_sensor_task(void *pvParameter) 
{
  Serial.begin(9600);
  pinMode(LED32, OUTPUT);
  
  while(1) 
  {
    sensor_Aout = digitalRead(GAS_SENSOR_INPUT);  
    Blynk.virtualWrite(V0, sensor_Aout);

    Serial.print("\nGas Sensor: ");  
    Serial.print(sensor_Aout);  
    Serial.print("\t");

    if (sensor_Aout > 1800) {  
      Serial.println("Gas detected");  
      digitalWrite (LED32, HIGH) ; /*LED set HIGH if Gas detected */
    }
    else {
      Serial.println("No Gas");
      digitalWrite (LED32, LOW) ;  /*LED set LOW if NO Gas detected */
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

}

float duration_us, distance_cm;
void ultrasonic_sensor_task(void *pvParameter)
{
 // initialize serial port
  Serial.begin(9600);
  // set ESP32 pins to output/input mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  while(1)
  {
  // generate 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    // calculate the distance
    distance_cm = 0.017 * duration_us;
    Blynk.virtualWrite(V1, distance_cm);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void ultrasonic_sensor_task_2(void *pvParameter){
  while(1)
  {
    if (distance_cm < DISTANCE_THRESHOLD)
    digitalWrite(BUZZER_PIN, HIGH); // turn on Piezo Buzzer
    else
    digitalWrite(BUZZER_PIN, LOW); // turn off Piezo Buzzer
    // print the value to Serial Monitor
    Serial.print("\ndistance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

int Signal;       
void heartbeat_sensor_task(void *pvParameter) 
{ 
  Serial.begin(9600);
  pinMode(LED21,OUTPUT);        
  while(1) 
  {
    Signal = analogRead(HEARTBEAT_SENSOR_PIN) / 37;  // Read the PulseSensor’s value.
    Blynk.virtualWrite(V2, Signal);
    Serial.print("\nHeartbeat: ");  
    Serial.print(Signal);                   
    if(Signal > HEARTBEAT_THRESHOLD)
    {                         
      digitalWrite(LED21,HIGH);
    } 
    else
    {
      digitalWrite(LED21,LOW);                //  Else, the sigal must be below “550”, so “turn-off” this LED.
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
 }

void setup()
{
  Blynk.begin(auth, ssid, pass);

  Serial.print("Hello");
  xTaskCreate(ultrasonic_sensor_task, "ultrasonic_sensor", 10000, NULL, 4, NULL);
  xTaskCreate(ultrasonic_sensor_task_2, "ultrasonic_sensor_2", 10000, NULL, 5, NULL);
  xTaskCreate(gas_sensor_task, "gas_sensor", 10000, NULL, 1, NULL);
  xTaskCreate(heartbeat_sensor_task, "heartbeat_sensor", 10000, NULL, 2, NULL);
  xTaskCreate(gps_sensor_task, "gps sensor", 10000, NULL, 3, NULL);
}

void loop()
{
  Blynk.run();
  timer.run();

  delay(5000);
}
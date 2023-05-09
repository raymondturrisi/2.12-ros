/*

For ESP32 UWB or ESP32 UWB Pro

*/
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define DEV_I2C Wire
#define SerialPort Serial
#define LedPin LED_BUILTIN

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);
Servo servo;
const char *ssid = "superuser_2ghz";
const char *password = "friyay2023";
const char *host = "192.168.1.6";
WiFiClient client;

int servoPin = 13; 
int index_num = 0;
long runtime = 0;
long max_dist = 3000;
int wait = 2000;
int sweep_time = 2000;
float sweep_angle =  60;
float angle_offset = 90-sweep_angle/2;
String all_json = "";
String range_status = "";
float obj;
float angle;
float distance;
int dist;
int mod;

void setup()
{
    // Led.
    pinMode(LedPin, OUTPUT);
    Serial.begin(115200);
    SerialPort.println("Starting...");
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());

    if (client.connect(host, 9002))
    {
        Serial.println("Success");
        client.print(String("GET /") + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     "Connection: close\r\n" +
                     "\r\n");
    }

    delay(1000);
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo.setPeriodHertz(50);// Standard 50hz servo
    servo.attach(servoPin, 500, 2400); // using FS90 servo min/max of 500us 
                                       // and 2400us, which are the defaults
    
    // Initialize I2C bus.
    DEV_I2C.begin();
  
    // Configure VL53L4CX satellite component.
    sensor_vl53l4cx_sat.begin();
  
    // Switch off VL53L4CX satellite component.
    sensor_vl53l4cx_sat.VL53L4CX_Off();
  
    //Initialize VL53L4CX satellite component.
    sensor_vl53l4cx_sat.InitSensor(0x12);
  
    // Start Measurements
    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

    servo.attach(26);
    
}

void loop(){ 
    // Set sweeping motion for servo
    mod = (millis() - wait) % sweep_time;
    angle = 2*mod*sweep_angle/sweep_time;
    angle = min(angle,2*sweep_angle-angle)+angle_offset;
    servo.write(angle);

    // Read data from ToF sensor
    VL53L4CX_MultiRangingData_t MultiRangingData;
    VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

    // initilaize serial data
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    char report[128];
    int status;

    
    do {
      status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
    } while (!NewDataReady);

    //Led on
    digitalWrite(LedPin, HIGH);

    if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

      // Find the closest object
      obj = max_dist;
      for (j = 0; j < no_of_object_found; j++) {
        dist = pMultiRangingData->RangeData[j].RangeMilliMeter;
        if (dist < obj){
          obj = dist;
          }
      }

      // Serializes data to send to client
      memset(report,sizeof(report),'\0');
      snprintf(report, sizeof(report), "{angle: %0.2f, distance: %0.2f}", angle-90, obj);
      SerialPort.print(report);
      SerialPort.println("");
      if (status == 0) {
        status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
      }
    }
  
    digitalWrite(LedPin, LOW);
    if ((millis() - runtime) > 100){
        String report_s = String(report);
        send_udp(&report_s);
        runtime = millis();
    }
}

void send_udp(String *msg_json)
{
    if (client.connected())
    {
        client.print(*msg_json);
        Serial.println("UDP send");
    }
}

#include <WiFi.h>                 // รวมไลบรารีสำหรับการเชื่อมต่อ WiFi
#include <PubSubClient.h>         // รวมไลบรารีสำหรับการเชื่อมต่อกับ MQTT broker
#include <NTPClient.h>            // รวมไลบรารีสำหรับการเชื่อมต่อกับ NTP server เพื่อดึงเวลา
#include <WiFiUdp.h>              // รวมไลบรารีสำหรับ UDP ที่ใช้ร่วมกับ NTPClient

#include <Adafruit_Sensor.h>      // รวมไลบรารีสำหรับเซ็นเซอร์ของ Adafruit
#include <DHT.h>                  // รวมไลบรารีสำหรับเซ็นเซอร์ DHT
#include <DHT_U.h>                // รวมไลบรารีสำหรับการใช้ฟังก์ชันเพิ่มเติมของ DHT

#include <Wire.h>                 // รวมไลบรารีสำหรับการสื่อสาร I2C
#include <Max44009.h>             // รวมไลบรารีสำหรับเซ็นเซอร์วัดแสง Max44009

#include <SoftwareSerial.h>       // รวมไลบรารีสำหรับการสื่อสารแบบ serial โดยใช้ขา GPIO
#include <ModbusMaster.h>         // รวมไลบรารีสำหรับการสื่อสารแบบ Modbus

// การตั้งค่า WiFi
const char *ssid = "rasberryPi";          // ชื่อ WiFi ที่จะเชื่อมต่อ
const char *password = "Noname6547";      // รหัสผ่านของ WiFi

WiFiClient espClient;                     // สร้างคลาส WiFiClient สำหรับการสื่อสาร WiFi
PubSubClient client(espClient);           // สร้างคลาส PubSubClient โดยใช้ WiFiClient สำหรับการเชื่อมต่อ MQTT

// การตั้งค่า MQTT
const char *mqtt_server = "192.168.1.102"; // ที่อยู่ของ MQTT broker
const int mqtt_port = 1883;                // พอร์ตของ MQTT broker
const char *mqtt_user = "";                // ชื่อผู้ใช้ MQTT (หากมี)
const char *mqtt_password = "";            // รหัสผ่าน MQTT (หากมี)
const char *sensorDataTopic = "sensor/prediction"; // หัวข้อ MQTT ที่จะใช้ส่งข้อมูลเซ็นเซอร์

// การตั้งค่า PZEM 017
SoftwareSerial PZEMSerial;                 // สร้าง SoftwareSerial สำหรับ PZEM-017

#define MAX485_RO  16                      // กำหนดขา RO สำหรับ RS485
#define MAX485_RE  4                       // กำหนดขา RE สำหรับ RS485
#define MAX485_DE  2                       // กำหนดขา DE สำหรับ RS485
#define MAX485_DI  17                      // กำหนดขา DI สำหรับ RS485

// กำหนดที่อยู่ของ PZEM-017 (0x01-0xF7)
static uint8_t pzemSlaveAddr = 0x01;

// กำหนดค่าของชั้นชั่งน้ำหนัก shunt (ที่ 50A)
static uint16_t NewshuntAddr = 0x0001;

ModbusMaster node;                         // สร้างคลาส ModbusMaster สำหรับการสื่อสาร Modbus

float PZEMVoltage, PZEMCurrent, PZEMPower, PZEMEnergy; // ตัวแปรสำหรับเก็บข้อมูลไฟฟ้าจาก PZEM

#define DHTPIN 18                          // กำหนดขา GPIO ที่เชื่อมต่อกับ DHT
#define DHTTYPE DHT22                      // กำหนดชนิดเซ็นเซอร์ DHT

DHT dht(DHTPIN, DHTTYPE);                  // สร้างคลาส DHT สำหรับเซ็นเซอร์ DHT

Max44009 lightSensor(0x4A);                // สร้างคลาส Max44009 สำหรับเซ็นเซอร์แสง

WiFiUDP ntpUDP;                            // สร้างคลาส UDP สำหรับใช้กับ NTPClient
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // ตั้งค่า NTPClient ซิงค์เวลา UTC+7 ทุกๆ 30 วินาที

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);              // เชื่อมต่อกับ WiFi

  while (WiFi.status() != WL_CONNECTED) {  // รอให้เชื่อมต่อ WiFi เสร็จสิ้น
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());          // แสดง IP address ของ ESP32
}

void reconnect() {
  while (!client.connected()) {            // เช็คการเชื่อมต่อกับ MQTT broker
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX); // สร้าง client ID สำหรับ MQTT
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");           // ถ้าเชื่อมต่อสำเร็จ
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());          // แสดงรหัสการเชื่อมต่อที่ล้มเหลว
      Serial.println(" try again in 5 seconds");
      delay(5000);                           // รอ 5 วินาทีและลองใหม่
    }
  }
}

void setup() {
  Serial.begin(115200);                     // เริ่ม serial communication
  PZEMSerial.begin(9600, SWSERIAL_8N2, MAX485_RO, MAX485_DI); // เริ่ม serial สำหรับ PZEM-017

  pinMode(MAX485_RE, OUTPUT);               // กำหนดขา RE เป็น output สำหรับ RS485
  pinMode(MAX485_DE, OUTPUT);               // กำหนดขา DE เป็น output สำหรับ RS485
  digitalWrite(MAX485_RE, 0);               // ปิดสัญญาณที่ขา RE
  digitalWrite(MAX485_DE, 0);               // ปิดสัญญาณที่ขา DE

  node.preTransmission(preTransmission);    // ตั้งค่า callback สำหรับ preTransmission
  node.postTransmission(postTransmission);  // ตั้งค่า callback สำหรับ postTransmission
  node.begin(pzemSlaveAddr, PZEMSerial);    // เริ่มการสื่อสาร Modbus กับ PZEM

  setup_wifi();                             // เรียกฟังก์ชันเชื่อมต่อ WiFi

  client.setServer(mqtt_server, mqtt_port); // ตั้งค่า MQTT server

  dht.begin();                              // เริ่มการทำงานของเซ็นเซอร์ DHT

  Wire.begin(21, 22);                       // เริ่ม I2C ที่ขา SDA=21, SCL=22

  timeClient.begin();                       // เริ่มการทำงานของ timeClient
}

void loop() {
  timeClient.update();                      // อัปเดตเวลาจาก NTP server

  if (!client.connected()) {                // เช็คการเชื่อมต่อ MQTT
    reconnect();                            // ถ้าไม่เชื่อมต่อให้ลองใหม่
  }

  client.loop();                            // ให้ MQTT client ทำงาน

  int currentSecond = timeClient.getSeconds(); // ดึงวินาทีปัจจุบันจาก NTP

  if (currentSecond == 0 || currentSecond % 5 == 0) { // ทุก 5 วินาที
    float lux = lightSensor.getLux();                 // อ่านค่าความสว่างจากเซ็นเซอร์แสง
    float irr = (lux * 0.0079);                       // แปลงค่า lux เป็น irradiance
    float humidity = dht.readHumidity();              // อ่านค่าความชื้น
    float temperature = dht.readTemperature();        // อ่านค่าอุณหภูมิ

    uint8_t result;                                   // สถานะของ Modbus
    result = node.readInputRegisters(0x0000, 6);      // อ่านค่าแรงดัน กระแส และพลังงานจาก PZEM

    if (result == node.ku8MBSuccess) {                // เช็คว่าการอ่านสำเร็จ
      uint32_t tempdouble = 0x00000000;               // ตัวแปรชั่วคราวสำหรับกำลังไฟฟ้า
      PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0; // อ่านแรงดัน
      PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0; // อ่านกระแส

      tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); // อ่านกำลังไฟฟ้า
      PZEMPower = tempdouble / 10.0;
    } else {                                          // ถ้าอ่านไม่สำเร็จ กำหนดเป็น NAN
      PZEMVoltage = NAN;
      PZEMCurrent = NAN;
      PZEMPower = NAN;
    }

    // สร้างข้อมูล JSON พร้อม timestamp จาก NTP server
    String jsonString = "{\"timestamp\":\"" + timeClient.getFormattedTime() + "\","
                      + "\"temperature\":" + String(temperature, 2) + 
                      ",\"humidity\":" + String(humidity, 1) + 
                      ",\"irradiance\":" + String(irr, 2) + 
                      ",\"voltage\":" + String(PZEMVoltage, 2) +
                      ",\"current\":" + String(PZEMCurrent, 2) + 
                      ",\"power\":" + String(PZEMPower, 2) + "}";

    char jsonCharArray[jsonString.length() + 1];
    jsonString.toCharArray(jsonCharArray, jsonString.length() + 1);

    // ส่งข้อมูลไปที่ MQTT broker
    client.publish(sensorDataTopic, jsonCharArray);

    // แสดงผลข้อมูลใน Serial Monitor สำหรับ debugging
    Serial.println("Published data:");
    Serial.println(jsonString);
    Serial.println("---");

    // รอให้เวลาขยับไปก่อนที่จะส่งครั้งถัดไป (เพื่อป้องกันการส่งซ้ำหลายครั้งใน 1 วินาที)
    delay(3000);
  } 
}

void preTransmission() {
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

    // สร้างข้อมูล JSON พร้อม timestamp จาก NTP server
    String jsonString = "{\"timestamp\":\"" + timeClient.getFormattedTime() + "\","
                      + "\"temperature\":" + String(temperature, 2) + 
                      ",\"humidity\":" + String(humidity, 1) + 
                      ",\"irradiance\":" + String(irr, 2) + 
                      ",\"voltage\":" + String(PZEMVoltage, 2) +
                      ",\"current\":" + String(PZEMCurrent, 2) + 
                      ",\"power\":" + String(PZEMPower, 2) + "}";

    char jsonCharArray[jsonString.length() + 1];
    jsonString.toCharArray(jsonCharArray, jsonString.length() + 1);

    // ส่งข้อมูลไปที่ MQTT broker
    client.publish(sensorDataTopic, jsonCharArray);

    // แสดงผลข้อมูลใน Serial Monitor สำหรับ debugging
    Serial.println("Published data:");
    Serial.println(jsonString);
    Serial.println("---");

    // รอให้เวลาขยับไปก่อนที่จะส่งครั้งถัดไป (เพื่อป้องกันการส่งซ้ำหลายครั้งใน 1 วินาที)
    delay(3000);
  } 
}

void preTransmission() {
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

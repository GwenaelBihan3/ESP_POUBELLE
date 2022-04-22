/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LIGHT_SENSOR_PIN  34  // ESP32 pin GIOP36 (ADC0) connected to light sensor
#define TILT_SENSOR_PIN  35  // ESP32 pin GIOP36 (ADC0) connected to tilt sensor
#define GYRO_SENSOR_PIN1  33  // ESP32 pin GIOP36 (ADC0) connected to tilt sensor
#define GYRO_SENSOR_PIN2  32 // ESP32 pin GIOP36 (ADC0) connected to tilt sensor



// WiFi network name and password:
const char* networkName = "AndroidAP6398";
const char* networkPswd = "aaaaaaaa";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char* udpAddress = "192.168.43.57";
const int udpPort = 2205;

int input;
//Are we currently connected?
boolean connected = false;

char packetBuffer[255]; //buffer to hold incoming packet


Adafruit_MPU6050 mpu;
//The udp library class
WiFiUDP udp;

void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

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

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
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
    break;
  }

  Serial.println("");
  delay(100);
  //pinMode(LED_PIN, OUTPUT); // set ESP32 pin to output mode
  //pinMode(BLUE_LED_PIN, OUTPUT); // set ESP32 pin to output mode


  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop() {
  int analogLight = analogRead(LIGHT_SENSOR_PIN);
  int analogTilt = analogRead(TILT_SENSOR_PIN);
  int analogGyro1 = analogRead(GYRO_SENSOR_PIN1);
  int analogGyro2 = analogRead(GYRO_SENSOR_PIN2);


  Serial.println("Tilt Value = ");
  Serial.println(analogTilt);  // the raw analog reading
  delay(2000);

  Serial.println("Light Value = ");
  Serial.println(analogLight);  // the raw analog reading
  delay(2000);

  Serial.println("Gyro Value 1 = ");
  Serial.println(analogGyro1);  // the raw analog reading
  delay(2000);

  Serial.println("Gyro Value 2= ");
  Serial.println(analogGyro2);  // the raw analog reading
  delay(2000);

  if (analogLight < 40) {
    Serial.println(" => Dark");
  } else if (analogLight < 800) {
    Serial.println(" => Dim");
  } else if (analogLight < 2000) {
    Serial.println(" => Light");
  } else if (analogLight < 3200) {
    Serial.println(" => Bright");
  } else {
    Serial.println(" => Very bright");
  }
  delay(2000);
  
  if (connected) {
    udp.beginPacket(udpAddress, udpPort);
    udp.printf("Seconds since boot: %u", millis() / 1000);
    IPAddress remoteIp = udp.remoteIP();
    const uint8_t buffer[50] = "test world";
    //This initializes udp and transfer buffer
    udp.beginPacket(udpAddress, udpPort);
    udp.write(buffer, 11);
    udp.endPacket();
    //static_cast<void*>(memset(buffer, 0, 50) ;
    //processing incoming packet, must be called before reading the buffer
    udp.parsePacket();
    //receive response from server, it will be HELLO WORLD
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
    //Wait for 1 second
    delay(1000);
    /*if (Serial.available()) {
      //input = Serial.readStringUntil('\n');
      //Serial.println("input, " + input + "!");
      input = Serial.parseInt();
      Serial.println(input);
      //digitalWrite(BLUE_LED_PIN, input);  // Set GPIO22 active high

    }*/
  }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(2000);}

void connectToWiFi(const char* ssid, const char* pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
  }
}
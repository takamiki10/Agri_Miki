#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <M5Stack.h>
#include <WiFi.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

#define DYNAMIXEL_SERIAL Serial2 
#define DYNAMIXEL_BAUDRATE 1000000
Dynamixel2Arduino dxl(DYNAMIXEL_SERIAL);

WiFiClient client;
IPAddress server(172, 20, 10, 2);

void disableTorqueControl() {
  for (uint8_t id = 2; id <= 5; id++) {
    dxl.torqueOff(id);
  }
  M5.Lcd.printf("Torque Control Turned Off\n");
}

class WiFiHardware {
public:
  WiFiHardware(){};
  void init() {
    client.connect(server, 11411);
  }
  int read() {
    return client.read();
  }
  void write(uint8_t* data, int length) {
    client.write(data, length);
  }
  unsigned long time() {
    return millis();
  }
};

void setupWiFi() {
  M5.Lcd.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    M5.Lcd.println("Could not connect to WiFi");
    while (1) delay(500);
  }
  M5.Lcd.println("WiFi connected");
}

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::Float32MultiArray float_array_msg;
ros::Publisher pub("float_array", &float_array_msg);

void callback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length < 4) {
    return;
  }

  dxl.setGoalVelocity(2, msg.data[0]); 
  dxl.setGoalVelocity(3, msg.data[1]); 
  dxl.setGoalVelocity(4, msg.data[2]); 
  dxl.setGoalVelocity(5, msg.data[3]); 
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("float_array", &callback);

void setup() {
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);
  setupWiFi();
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  DYNAMIXEL_SERIAL.begin(DYNAMIXEL_BAUDRATE);
  dxl.begin(DYNAMIXEL_SERIAL);

  for (uint8_t id = 2; id <= 6; id++) {
    if (dxl.ping(id)) {
      M5.Lcd.printf("Dynamixel with ID %d initialized successfully\n", id);
    } else {
      M5.Lcd.printf("Failed to initialize Dynamixel with ID %d\n", id);
    }
  }
  disableTorqueControl();
  for(uint8_t id = 2; id <= 5; id++){
    dxl.setOperatingMode(id, 1);
  }
}

void loop() {
  nh.spinOnce();
  delay(1);
}

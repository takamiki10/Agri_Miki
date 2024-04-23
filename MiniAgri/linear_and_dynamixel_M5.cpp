//DYNAMIXEL ID 2 and 3 on the RIGHT side, and ID 4 and 5 on the LEFT side

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <M5Stack.h>
#include <WiFi.h>

//========================================================//


#define x_radius              16.5
#define y_radius              25.0
#define yaw_gear_ratio        2.50
#define pitch_gear_ratio      2.31

#define MAX_VELOCITY          500
#define OP_POSITION           3
#define OP_VELOCITY           1
#define OP_EXTENDED_POSITION  4

//Setup Wifi
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
WiFiClient client;
IPAddress server(172, 20, 10, 2);

//Setup Dynamixel
#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = 0; //When using wireless, set to 0
const float DXL_PROTOCOL_VERSION_AX = 1.0;
const float DXL_PROTOCOL_VERSION_X = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
#define startID         2
#define endID           6


//Setup Linear Actuator
int pwm_pin=13;
int dir_pin=14;
const int CHANNEL_0 = 0;
const int LEDC_TIMER_BIT = 8;
const int LEDC_BASE_FREQ = 12000; 

//========================================================//

void disableTorqueControl() {
  for (uint8_t id = startID ; id <= endID; id++) {
    dxl.torqueOff(id);
  }
}

void enableTorqueControl(){
  for (uint8_t id = startID ; id <= endID; id++) {
    dxl.torqueOn(id);
  }
}

void setOperatingModes(int op){
  for (uint8_t id = startID ; id <= endID; id++) {
    dxl.setOperatingMode(id,op);
  }
}

void setMaxVelocity(){
  for (uint8_t id = startID ; id <= endID; id++) {
    dxl.writeControlTableItem(44,id,MAX_VELOCITY);
  }
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

//========================================================//

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

//========================================================//

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::Float32MultiArray float_array_msg;
ros::Publisher pub("float_array", &float_array_msg);

void callback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length < 5) {
    return;
  }
  for(int i = 0; i <= startID - endID;i++){
    dxl.setGoalVelocity(i+startID, msg.data[i]);
  } 

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("float_array", &callback);//[Y GOAL VELOCITY, X GOAL VELOCITY, YAW GOAL VELOCITY, PITCH GOAL VELOCITY]

//========================================================//

void setup() {

  pinMode(pwm_pin,OUTPUT);
  pinMode(dir_pin,OUTPUT);

  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);
  ledcSetup(CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(pwm_pin, CHANNEL_0);

  setupWiFi();
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  dxl.begin(1000000);
  Serial2.begin(1000000, SERIAL_8N1, 33, 32);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION_X);

  disableTorqueControl();
  setOperatingModes(OP_VELOCITY);
  setMaxVelocity();
  enableTorqueControl();

}

void loop() {
  nh.spinOnce();
  delay(1);
}

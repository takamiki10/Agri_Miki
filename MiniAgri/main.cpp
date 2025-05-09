#include <Arduino.h>
#include <M5Core2.h>
#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <WiFi.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h> 
#include <math.h>

using std::to_string;

float distance_angle_conv(float distance, float radius);
float x_radius = 16.5;
float y_radius = 25.0;
float yaw_gear_ratio = 2.05;//Yawギア比 52:24
float pitch_gear_ratio = 2.5;//Pitchギア比 60:24
float init_x_pos;
float init_yaw_pos;
float init_pitch_pos;
float x_pos_state=0;
float yaw_pos_state=0;
float pitch_pos_state=0;
double radian_yaw;
double degree_yaw;
double move_y ;
double x_t;
double y_t;
double x_t_scaled;
double y_t_scaled;
double depth_scale;
int z_t;//USDM
int Zdistance; //[cm]      
int start_pos = 25;//リニア初期位置
int flag_xyz = 0;
int flag_initpos = 0;
int flag_touch = 0;
int x_correction = 370;
int y_correction = 365;
int arm_length = 183;//アーム中心軸からハンド先端部までのxy平面上での距離

//超音波測距　ここから　USDM
unsigned long int currentTime;
unsigned long int ptime;
long pulselength; //[usec]
const double soundvelocity = 34350./1000000.; //[cm/usec at 20 degrees Celsius]
const int HCTrigPin = 14; //超音波測距で使用するGPIOピン
const int HCEchoPin = 13; //超音波測距で使用するGPIOピン
long USechotime()  //[usec]
{
  digitalWrite(HCTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(HCTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCTrigPin,LOW);
  long echotime = pulseIn(HCEchoPin, HIGH); //[usec] 指示しないのでTimeoutは1秒
  return echotime;
}
//超音波測距　ここまで　USDM

int pwm_pin = 19;
int dir_pin = 27;
const int CHANNEL_0 = 0;
const int LEDC_TIMER_BIT = 8; // PWM の範囲(8bit なら 0〜255，10bit なら 0〜1023)
const int LEDC_BASE_FREQ = 312500;// 8bitの場合、最大周波数 312500Hz

// 使用するWi-FiのSSIDとパスワードを入力する
  //const char *ssid = "daichi_iPhone15";
  //const char *password = "aaaaaaaa";
  //const char *ssid = "Aoi";//shimada
  //const char *password = "aoshima0611";
  //const char *ssid = "WIN-KT6M9JJLDTL 4378";
  //const char *password = "9]V17d62";
  //const char *ssid = "Buffalo-G-3C0E";
  //const char *password = "kbdtuihtxdsx3";
  //const char *ssid = "HUAWEI nova";
  //const char *password = "653e0d5d428c";
  //const char *ssid = "DESKTOP-QVOJTML 7772";
  //const char *password = "M91k2]6";
  //const char *ssid = "F660P-3dtQ-G";
  //const char *password = "fz6sb6k9f42xs";
  //const char *ssid = "Galaxy_5GMW_1585";
  //const char *password = "zmya4157h";
  //const char *ssid = "A101ZTa-775227";//Agri wi-fi
  //const char *password = "0079541a";
  // const char *ssid = "wathlete_wifi";//wathlete wi-fi
  // const char *password = "7532AtsuO";
  const char *ssid = "AiR-WiFi_0TZOKM";
const char *password = "85075670"; //otaniLabMobile

#define DXL_SERIAL Serial2
const uint8_t DXL_DIR_PIN = 0; // 無線通信では使用しないため0とする
const float DXL_PROTOCOL_VERSION_AX = 1.0;
const float DXL_PROTOCOL_VERSION_X = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;
WiFiClient client;
// ROSのIPアドレスを設定
  //IPAddress server(172, 20, 10, 4);//shimada
  //IPAddress server(192, 168, 137, 202);
  //IPAddress server(192, 168, 9, 224);
  //IPAddress server(192, 168, 43, 84);
  //IPAddress server(192, 168, 43, 45);
  //IPAddress server(192, 168, 137, 162);
  //IPAddress server(192, 168, 128, 108);
  //IPAddress server(172,20,10,9);
  //IPAddress server(192,168,128,201);
//IPAddress server(192,168,128,108);//miniagri-pc
//IPAddress server(192,168,43,209);//miniagri-pc
IPAddress server(192,168,43,107);//auto-takanishi-pc
TFT_eSprite img = TFT_eSprite(&M5.Lcd);

class WiFiHardware
{
  public:
  WiFiHardware() {};
  void init()
  {
    client.connect(server, 11411);
  }
  int read()
  {
    return client.read();
  }
  void write(uint8_t *data, int length)
  {
    for (int i = 0; i < length; i++)
      client.write(data[i]);
  }
  unsigned long time()
  {
    return millis(); // easy; did this one for you
  }
};

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::Float64 task_state_msg;//shimada add 10/20
ros::Publisher task_state_pub("task_state", &task_state_msg);//shimada add 10/20

void Move_to_target(int z_t, int& Zdistance) {
  Zdistance=0;//Zdistance 初期化
  while (abs(z_t - Zdistance) !=0)//目標値に達したらwhile文外へ
  {
    currentTime = micros();
    if ( currentTime - ptime > 80000) 
    {
      pulselength = USechotime(); //[usec]
      Zdistance = pulselength * soundvelocity / 2.;
      M5.Lcd.setTextSize(4);
      M5.Lcd.setCursor(0, 160);
      M5.Lcd.printf("%d cm\n",Zdistance );  //測距値表示
      M5.Lcd.setCursor(0, 200);
      M5.Lcd.printf("z_target %d\n",z_t ); //目標値表示
      ptime = currentTime; 
    }
    
    if(z_t > Zdistance )
    {
      digitalWrite(dir_pin, HIGH);//Extending
      ledcWrite(CHANNEL_0, 255);
    }
    
    else if(z_t < Zdistance )
    {
      digitalWrite(dir_pin, LOW);//Shrinking
      ledcWrite(CHANNEL_0, 255);
    }
  }
    digitalWrite(dir_pin, LOW);//stop
    ledcWrite(CHANNEL_0, 0);
    
}

void touchCb(const std_msgs::Float32 &touch)
{
  // 受信したデータを表示します
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Touch data received: %f\n", touch.data);
  flag_touch = 1;

}

void messageCb(const std_msgs::Float64MultiArray &command)
{

  x_t = command.data[0]-x_correction;         //中心を原点に変更
  y_t = -(command.data[1])+y_correction;      //中心を原点に変更
  z_t = command.data[2]*0.0878-7.0128+start_pos;
  flag_initpos = command.data[3];
  
  // 深度に基づいてXY補正を適用
  depth_scale = command.data[2]/600.0;  // 基準深度600mmを基準に補正   基準深度：100pixel=100mmになる高さ
  x_t_scaled = x_t * depth_scale;
  y_t_scaled = y_t * depth_scale;
  
  if(y_t_scaled>=0)
  {
    radian_yaw = asin((x_t_scaled) / arm_length);
    degree_yaw = radian_yaw * (180.0 / M_PI); //角度に変換
    move_y = (y_t_scaled) - arm_length * cos(radian_yaw);
  }
  
  else if(y_t_scaled<0&&x_t_scaled>=0)
  {
    radian_yaw = asin((x_t_scaled) / arm_length);
    degree_yaw = 180-(radian_yaw * (180.0 / M_PI)); //角度に変換
    move_y = (y_t_scaled) + arm_length * cos(radian_yaw);
  }
  
  else if(y_t_scaled<0&&x_t_scaled<0)
  {
    radian_yaw = asin((x_t_scaled) / arm_length);
    degree_yaw = -180-(radian_yaw * (180.0 / M_PI)); //角度に変換
    move_y = (y_t_scaled) + arm_length * cos(radian_yaw);
  }
  
  // 表示 (デバッグ用)
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("depth_scale: %5.2f\n", depth_scale);
  M5.Lcd.setCursor(0, 52);
  M5.Lcd.printf("y_scaled: %5.2f\n", y_t_scaled);
  M5.Lcd.setCursor(0, 84);
  M5.Lcd.printf("degree_yaw: %lf\n", degree_yaw);
  M5.Lcd.setCursor(0, 106);
  M5.Lcd.printf("move_y: %5.2f\n", move_y);
  
  if(abs(x_t_scaled)>arm_length||move_y>310||move_y<-180)//可動範囲外警告
  {
    M5.Lcd.setTextSize(4);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("out of range\n");
    delay(3000);
  }

  else
  {
    flag_xyz = 1;
  }

}

ros::Subscriber<std_msgs::Float64MultiArray> sub("command", &messageCb);
ros::Subscriber<std_msgs::Float32> touch_sub("touch", &touchCb);

void setupWiFi()
{
  WiFi.begin(ssid, password);
  M5.Lcd.printf("\nConnecting to ");
  // Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20)
    delay(500);
  if (i == 21)
  {
    M5.Lcd.printf("Could not connect to");
    // Serial.println(ssid);1
    while (1)
      delay(500);
  }
  M5.Lcd.printf("Ready! Use ");
  // Serial.print(WiFi.localIP());
  M5.Lcd.printf(" to access client");
}

// Refactored setup() with full peripheral support and safe Dynamixel communication

void setup() {
  Serial.begin(115200);
  M5.begin();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Initializing...");

  // Initialize GPIOs
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(HCTrigPin, OUTPUT);
  pinMode(HCEchoPin, INPUT);

  ledcSetup(CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(pwm_pin, CHANNEL_0);

  // === Initialize Serial2 BEFORE Dynamixel ===
  Serial2.begin(1000000, SERIAL_8N1, 33, 32); // TX=33, RX=32 for Grove Port C
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION_X);
  delay(500);

  // === Ping Motors Safely ===
  int ids[] = {1, 4, 5, 6, 7};
  const int num_ids = sizeof(ids) / sizeof(ids[0]);
  bool is_alive[num_ids] = {false};

  M5.Lcd.println("Pinging motors...");
  for (int i = 0; i < num_ids; i++) {
    unsigned long start = millis();
    while (millis() - start < 500) {
      if (dxl.ping(ids[i])) {
        is_alive[i] = true;
        break;
      }
      delay(10);
    }
    M5.Lcd.setCursor(0, 40 + i * 20);
    if (is_alive[i]) M5.Lcd.printf("ID %d: Found \xE2\x9C\x93", ids[i]);
    else M5.Lcd.printf("ID %d: Not Found \xE2\x9C\x97", ids[i]);
  }

  // === Configure Alive Motors ===
  for (int i = 0; i < num_ids; i++) {
    if (!is_alive[i]) continue;
    dxl.torqueOff(ids[i]);
    if (ids[i] <= 5) dxl.setOperatingMode(ids[i], 4); // Extended pos ctrl
    else dxl.setOperatingMode(ids[i], 5); // Current-based pos ctrl
    dxl.torqueOn(ids[i]);
  }

  // === Safe Initial Position Read ===
  if (is_alive[0]) init_x_pos = dxl.getPresentPosition(1, UNIT_DEGREE);
  if (is_alive[1]) init_yaw_pos = dxl.getPresentPosition(4, UNIT_DEGREE);
  if (is_alive[2]) init_pitch_pos = dxl.getPresentPosition(5, UNIT_DEGREE);

  // === Safe Velocity Write ===
  if (is_alive[0]) dxl.writeControlTableItem(PROFILE_VELOCITY, 1, 150);
  if (is_alive[1]) dxl.writeControlTableItem(PROFILE_VELOCITY, 4, 100);
  if (is_alive[2]) dxl.writeControlTableItem(PROFILE_VELOCITY, 5, 100);
  if (is_alive[3]) dxl.writeControlTableItem(PROFILE_VELOCITY, 6, 100);
  if (is_alive[4]) dxl.writeControlTableItem(PROFILE_VELOCITY, 7, 100);

  // === Display Final Torque Status ===
  for (int i = 0; i < num_ids; i++) {
    if (!is_alive[i]) continue;
    int t = dxl.readControlTableItem(TORQUE_ENABLE, ids[i]);
    M5.Lcd.setCursor(180, 40 + i * 20);
    if (t == 1) M5.Lcd.printf("Torque \xE2\x9C\x93");
    else M5.Lcd.printf("Torque \xE2\x9C\x97");
  }

  // === Initialize Peripherals ===
  setupWiFi();
  delay(1000);
  img.createSprite(320, 240);
  img.fillSprite(TFT_BLACK);

  // === ROS Setup ===
  nh.initNode();
  nh.advertise(task_state_pub);
  nh.subscribe(sub);
  nh.subscribe(touch_sub);

  M5.Lcd.setCursor(0, 160);
  M5.Lcd.println("Setup Complete \xE2\x9C\x93");
}


void loop()
{

  // put your main code here, to run repeatedly:
  img.fillSprite(TFT_BLACK);
  img.pushSprite(0, 0);
  nh.spinOnce();
  M5.update();
  
  /*超音波測距関数をcallして結果を表示*/
  
  currentTime = micros();
  if ( currentTime - ptime > 200000) 
  {
  pulselength = USechotime(); //[usec]
  Zdistance = pulselength * soundvelocity / 2.;
    M5.Lcd.setTextSize(4);
    M5.Lcd.setCursor(0, 160);
    M5.Lcd.printf("%d cm\n",Zdistance );  //測距値表示
    M5.Lcd.setCursor(0, 200);
    M5.Lcd.printf("z_target %d\n",z_t ); //目標値表示
  ptime = currentTime; 
  }
  
  if(flag_xyz==1)
  {
    if(flag_initpos ==1)//初期位置に戻る
    {
      z_t = start_pos;
      Move_to_target(z_t, Zdistance);
      dxl.setGoalPosition(4, init_yaw_pos, UNIT_DEGREE);
      dxl.setGoalPosition(1, init_x_pos, UNIT_DEGREE);
      dxl.setGoalPosition(5, init_pitch_pos, UNIT_DEGREE);
    }
    
    else if(flag_initpos == 2)//補正モード
    {
        dxl.setGoalPosition(4, degree_yaw* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        dxl.setGoalPosition(5, 30 * pitch_gear_ratio + init_pitch_pos, UNIT_DEGREE);//Pitch
        dxl.setGoalPosition(1, -distance_angle_conv(move_y, x_radius) + init_x_pos, UNIT_DEGREE);//Y

        z_t = 50;
        Move_to_target(z_t, Zdistance);
        delay(3000);

        z_t = start_pos+20;
        Move_to_target(z_t, Zdistance);
      

    }
    
    else if(flag_initpos == 3)//Yaw振動
    {
        dxl.setGoalPosition(4, degree_yaw* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        dxl.setGoalPosition(5, 30 * pitch_gear_ratio + init_pitch_pos, UNIT_DEGREE);//Pitch
        dxl.setGoalPosition(1, -distance_angle_conv(move_y, x_radius) + init_x_pos, UNIT_DEGREE);//Y
        
          digitalWrite(dir_pin, HIGH);//Extending
          ledcWrite(CHANNEL_0, 255);
          delay(1000);
          
        while(flag_touch == 0)
        {
          nh.spinOnce();
          digitalWrite(dir_pin, HIGH);//Extending
          ledcWrite(CHANNEL_0, 255);
        }
        
        digitalWrite(dir_pin, LOW);//Shrinking
        ledcWrite(CHANNEL_0, 255);
        
        delay(1000);
        
        digitalWrite(dir_pin, LOW);//stop
        ledcWrite(CHANNEL_0, 0);
        
        
        //hand_close
        delay(1000);
        dxl.setGoalPosition(6, 15, UNIT_DEGREE);
        dxl.setGoalPosition(7, 300, UNIT_DEGREE);
        //dxl.setGoalPosition(2, 125, UNIT_DEGREE);

        dxl.writeControlTableItem(PROFILE_VELOCITY, 4, 10);
        delay(1000);
        dxl.setGoalPosition(4, (degree_yaw+3)* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        delay(500);
        dxl.setGoalPosition(4, (degree_yaw-3)* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        delay(500);
        dxl.setGoalPosition(4, (degree_yaw+3)* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        delay(500);
        dxl.setGoalPosition(4, (degree_yaw-3)* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        delay(500);
        dxl.setGoalPosition(4, (degree_yaw)* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        delay(1000);
        dxl.writeControlTableItem(PROFILE_VELOCITY, 4, 100);
        z_t = start_pos+15;
        Move_to_target(z_t, Zdistance);


        dxl.setGoalPosition(4, init_yaw_pos, UNIT_DEGREE);
        dxl.setGoalPosition(1, -distance_angle_conv(200, x_radius) + init_x_pos, UNIT_DEGREE);
        dxl.setGoalPosition(5, init_pitch_pos, UNIT_DEGREE);
        
        int release = (310-move_y)*30+1000;
        delay(release);

        dxl.setGoalPosition(5, 40 * pitch_gear_ratio + init_pitch_pos, UNIT_DEGREE);
        delay(2000);
        dxl.setGoalPosition(6, 95, UNIT_DEGREE);
        dxl.setGoalPosition(7, 120, UNIT_DEGREE);
        //dxl.setGoalPosition(2, 165, UNIT_DEGREE);
        delay(2000);
        dxl.setGoalPosition(5, init_pitch_pos, UNIT_DEGREE);
        
      
    }

    else
    {
        dxl.setGoalPosition(4, degree_yaw* yaw_gear_ratio + init_yaw_pos, UNIT_DEGREE);//Yaw
        dxl.setGoalPosition(5, 30 * pitch_gear_ratio + init_pitch_pos, UNIT_DEGREE);//Pitch
        dxl.setGoalPosition(1, -distance_angle_conv(move_y, x_radius) + init_x_pos, UNIT_DEGREE);//Y

        /*
          digitalWrite(dir_pin, HIGH);//Extending
          ledcWrite(CHANNEL_0, 255);
          delay(1000);
          
        while(flag_touch == 0)
        {
          nh.spinOnce();
          digitalWrite(dir_pin, HIGH);//Extending
          ledcWrite(CHANNEL_0, 255);
        }
        
        digitalWrite(dir_pin, LOW);//Shrinking
        ledcWrite(CHANNEL_0, 255);
        
        delay(1000);
        
        digitalWrite(dir_pin, LOW);//stop
        ledcWrite(CHANNEL_0, 0);
        */
        //z_t = z_t-4;        
        z_t = 55;
        Move_to_target(z_t, Zdistance);
        
        //hand_close
        delay(1000);
        dxl.setGoalPosition(6, 15, UNIT_DEGREE);
        dxl.setGoalPosition(7, 300, UNIT_DEGREE);
        //dxl.setGoalPosition(2, 125, UNIT_DEGREE);
        delay(1500);

        z_t = start_pos+20;
        Move_to_target(z_t, Zdistance);


        dxl.setGoalPosition(4, init_yaw_pos, UNIT_DEGREE);
        dxl.setGoalPosition(1, -distance_angle_conv(250, x_radius) + init_x_pos, UNIT_DEGREE);
        dxl.setGoalPosition(5, init_pitch_pos, UNIT_DEGREE);

        int release = (310-move_y)*30-4000;
        delay(release);

        dxl.setGoalPosition(5, 40 * pitch_gear_ratio + init_pitch_pos, UNIT_DEGREE);
        delay(1500);
        dxl.setGoalPosition(6, 95, UNIT_DEGREE);
        dxl.setGoalPosition(7, 120, UNIT_DEGREE);
        //dxl.setGoalPosition(2, 165, UNIT_DEGREE);
        delay(1500);
        dxl.setGoalPosition(5, init_pitch_pos, UNIT_DEGREE);
    }
    
    flag_xyz = 0;
    flag_touch = 0;
      
        task_state_msg.data=1;//shimada add 10/20
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.printf("Publishing task_state_msg: %d\n", task_state_msg.data);
        nh.spinOnce(); 
        task_state_pub.publish(&task_state_msg);//shimada add 10/20
        task_state_msg.data=0;//shimada add 10/20
        
  }
  
}

// 目標とするx,yの移動距離を回転角度に変換する関数
float distance_angle_conv(float distance, float radius)
{
  return (360 * distance) / (2 * M_PI * radius);
}


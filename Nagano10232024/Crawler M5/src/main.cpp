#include <Arduino.h>
#include <M5Core2.h>
#include <FloatArrayEncoder.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

void sendValues();
void IRAM_ATTR onTimer();  // Timer interrupt function
void callback(const std_msgs::Float64MultiArray &msg);  // ROS callback

hw_timer_t *timer = NULL;  // Hardware timer object
volatile bool sendCommand = false;  // Flag to indicate when to send a command

FloatArrayEncoder encoder;
float values[3] = {0.0, 0.0, 0.0}; 
uint8_t encoded_data[7]; // 7 bytes: 1 byte for header + 2 bytes per float (x3)

//----------------------------------------------------------------//

// Setup WiFi
const char* ssid = "A101ZTa-775227";
const char* password = "0079541a";
WiFiClient client;
IPAddress server(192, 168, 128, 209); // Replace with your ROS server IP

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);  // Connect to the ROS server on port 11411
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

ros::NodeHandle_<WiFiHardware> nh;  // Initialize ROS node handle with custom WiFi hardware
std_msgs::Float64MultiArray ros_values;
ros::Subscriber<std_msgs::Float64MultiArray> sub("crawlerCommand", callback);  // ROS subscriber to `crawlerCommand`

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

//----------------------------------------------------------------//

void setup() {
  M5.begin();
  Serial1.begin(9600, SERIAL_8N1, 32, 33);  // RX (32) and TX (33) pins for UART1
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.printf("Setup Complete");

  setupWiFi();

  // ROS setup
  nh.getHardware()->init();  // Initialize WiFi hardware for ROS communication
  nh.initNode();  // Initialize ROS node
  nh.subscribe(sub);  // Subscribe to ROS topic `crawlerCommand`

  // Setup timer to trigger interrupt every 5 seconds (5,000,000 microseconds)
  timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (for 1us ticks), count up
  timerAttachInterrupt(timer, &onTimer, true);  // Attach the interrupt to the timer
  timerAlarmWrite(timer, 5000000, true);  // Set alarm to 5 seconds
  timerAlarmEnable(timer);  // Enable the timer alarm

  delay(1000);  // Allow setup to complete before first interrupt
}

void loop() {
  M5.update();  // Update the status of the buttons
  nh.spinOnce();  // Handle ROS communication

  // Button A for stopping the movement
  if (M5.BtnA.isPressed() == 1) {
    values[0] = 0;  // Set values to stop
    sendValues();  // Send stop command
    M5.Lcd.clear();
    M5.Lcd.printf("STOP!!!");
  }

  // Timer-based sending (only send every 5 seconds)
  if (sendCommand) {
    sendValues();  // Send the values when the timer triggers
    sendCommand = false;  // Reset the flag
  }
}

// Timer interrupt service routine (ISR)
void IRAM_ATTR onTimer() {
  sendCommand = true;  // Set the flag to true when the timer fires
}

// ROS callback function for receiving Float64MultiArray
void callback(const std_msgs::Float64MultiArray &msg) {
  if (msg.data_length >= 3) {  // Ensure there are 3 float values
    values[0] = (float)msg.data[0];
    values[1] = (float)msg.data[1];
    values[2] = (float)msg.data[2];
    M5.Lcd.clear();
    M5.Lcd.printf("Received via ROS: %.2f, %.2f, %.2f", values[0], values[1], values[2]);
    sendValues();  // Send the received values via UART
  }
}

// Function to encode and send the current float values over UART1
void sendValues() {
  // Encode the 3 float values into the byte array
  encoder.encode(values, encoded_data);

  // Send the encoded byte array over UART1
  Serial1.write(encoded_data, sizeof(encoded_data));  // Send raw bytes

  // Display the encoded message on the Serial monitor for debugging
  Serial.print("Encoded data: ");
  for (int i = 0; i < sizeof(encoded_data); i++) {
    Serial.printf("%02X ", encoded_data[i]);  // Display encoded bytes in HEX format
  }
  Serial.println();
}

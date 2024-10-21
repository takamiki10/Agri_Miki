#include <Arduino.h>
#include <M5Core2.h>             // M5Core2 library for the LCD screen
#include <FloatArrayEncoder.h>   // Include your FloatArrayEncoder library

// Function Prototype
void sendValues();

// Create an instance of the FloatArrayEncoder class
FloatArrayEncoder encoder;

// Variables to hold the 3 float values
float values[3] = {0.0, 1.0, 2.0};  // Now using floats instead of ints

// Buffer to store the encoded data
uint8_t encoded_data[7];  // 7 bytes: 1 byte for header + 2 bytes per float (x3)

void setup() {
  // Initialize the M5Core2
  M5.begin();

  // Initialize UART communication for the M5 Stack using GPIO 32 (RX) and GPIO 33 (TX)
  Serial1.begin(9600, SERIAL_8N1, 32, 33);  // Set baud rate and specify RX (32) and TX (33) pins for UART1

  // Initialize Serial for debugging (UART0)
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial to be ready (especially useful for native USB)
  }

  // Clear the screen and set text size/color
  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);  // Set text size
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);  // White text on black background

  // Send an initial encoded message over UART1
  delay(1000);  // Delay for stability
  sendValues();  // Send the initial values (0, 1, 2)
}

void loop() {
  // Encode and send the current float values over UART1
  sendValues();

  // Display the sent values on the LCD screen
  M5.Lcd.fillRect(10, 50, 300, 40, TFT_BLACK);  // Clear previous text
  M5.Lcd.setCursor(10, 50);  // Set cursor position for new text
  M5.Lcd.printf("Sent: %.2f, %.2f, %.2f", values[0], values[1], values[2]);

  // Print the float array to the Serial terminal
  Serial.print("Float Array: ");
  Serial.print(values[0]);
  Serial.print(", ");
  Serial.print(values[1]);
  Serial.print(", ");
  Serial.println(values[2]);

  // Increment the float values by 1
  for (int i = 0; i < 3; i++) {
    values[i] += 1.0;
  }

  // Add a delay before sending the next set of values
  delay(5000);  // Send every 5 seconds
}

// Function to encode and send the current float values over UART1
void sendValues() {
  // Encode the 3 float values into the byte array
  encoder.encode(values, encoded_data);

  // Send the encoded byte array over UART1
  Serial1.write(encoded_data, sizeof(encoded_data));  // Send raw bytes

  // Display the encoded message on the LCD for confirmation
  M5.Lcd.fillRect(10, 10, 300, 40, TFT_BLACK);  // Clear previous text
  M5.Lcd.setCursor(10, 10);  // Set cursor position for new text
  M5.Lcd.print("UART communication: ");
  for (int i = 0; i < sizeof(encoded_data); i++) {
    M5.Lcd.printf("%02X ", encoded_data[i]);  // Display encoded bytes in HEX format
  }
}

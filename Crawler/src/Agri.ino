#include <Arduino.h>    // Include Arduino core for Serial functionality
#include "AgriCugoSDK.h"  // Include your custom SDK

void setup() {
  Serial.begin(115200);  // Initialize Serial communication
  cugo_switching_reset = false;  // Reset switching state
  cugo_init();  // Initialize CuGo settings
}

void loop() {
  if (cugo_runmode == CUGO_RC_MODE) {
    // Set control mode to remote control mode
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(100);  // Wait for 100ms
  } 
  else if (cugo_runmode == CUGO_CMD_MODE) {
    Serial.println(F("Starting autonomous mode"));
    cugo_wait(1000);  // Wait for 1 second

    Serial.println(F("Executing 1.0m square movement"));
    // Move forward 1.0 meters
    cugo_move_forward(1.0);
    cugo_wait(1000);  // Wait for 1 second

    // Rotate clockwise by 90 degrees
    cugo_turn_clockwise(90);
    cugo_wait(1000);

    // Move forward 1.0 meters again
    cugo_move_forward(1.0);
    cugo_wait(1000);

    // Rotate clockwise by 90 degrees
    cugo_turn_clockwise(90);
    cugo_wait(1000);

    // Continue the square movement
    cugo_move_forward(1.0);
    cugo_wait(1000);

    // Rotate clockwise by 90 degrees
    cugo_turn_clockwise(90);
    cugo_wait(1000);

    // Move forward 1.0 meters
    cugo_move_forward(1.0);
    cugo_wait(1000);

    // Final 90 degree clockwise turn to complete the square
    cugo_turn_clockwise(90);
    cugo_wait(1000);

    Serial.println(F("Autonomous mode finished"));
    
    // Switch back to remote control mode
    ld2_set_control_mode(CUGO_RC_MODE);
    cugo_wait(1000);  // Wait for 1 second
  }
}

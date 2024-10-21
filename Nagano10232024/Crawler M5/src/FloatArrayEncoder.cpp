#include "FloatArrayEncoder.h"

// Constructor (not doing much here, but added for completeness)
FloatArrayEncoder::FloatArrayEncoder() {
}

// Encode 3 float values into a byte array
void FloatArrayEncoder::encode(float* float_data, uint8_t* encoded_data) {
    encoded_data[0] = 0xFF;  // Set the header byte
    for(int i = 0; i < MOTOR_NUM; i++) {
        int data = int(float_data[i] * 100);  // Convert float to int with 2 decimal places
        int sign = (data >= 0) ? 1 : 0;  // Determine sign (1 for positive, 0 for negative)
        int value = abs(data);  // Get absolute value
        encoded_data[i*2+1] = (i << 6) | (sign << 5) | ((value >> 8) & 0x1F);  // Encode motor number, sign, and value (upper bits)
        encoded_data[i*2+2] = value & 0xFF;  // Encode lower bits of value
    }
}

// Decode a byte array into 3 float values
void FloatArrayEncoder::decode(uint8_t* encoded_data, float* data) {
    if(encoded_data[0] == 0xFF) {  // Check the header byte
        for(int i = 0; i < MOTOR_NUM; i++) {
            int motor_num = (encoded_data[i*2+1] & 0xC0) >> 6;  // Extract motor number
            if(i == motor_num) {
                int sign = (encoded_data[i*2+1] & 0x20) ? 1 : -1;  // Extract sign
                int value = ((encoded_data[i*2+1] & 0x1F) << 8) | encoded_data[i*2+2];  // Extract value
                data[i] = sign * float(value) / 100;  // Convert back to float
            }
        }
    }
}

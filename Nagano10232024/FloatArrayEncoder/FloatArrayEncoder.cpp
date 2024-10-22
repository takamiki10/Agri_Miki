#include "FloatArrayEncoder.h"
#include <Arduino.h>
#include <cmath> 

//This code will encode/decode max of +- 1,310.71 bits

FloatArrayEncoder::FloatArrayEncoder() {}

void FloatArrayEncoder::encode(float* float_data, uint8_t* encoded_data) {
    encoded_data[0] = 0xFF;  // Set the header byte
    for (int i = 0; i < MOTOR_NUM; i++) {
        int data = int(float_data[i] * 100);  
        int sign = (data >= 0) ? 1 : 0;  
        int value = abs(data); 
        
        encoded_data[i*3+1] = (i << 6) | (sign << 5) | ((value >> 12) & 0x1F);  
        encoded_data[i*3+2] = (value >> 4) & 0xFF; 
        encoded_data[i*3+3] = (value & 0x0F) << 4; 
    }
}

void FloatArrayEncoder::decode(uint8_t* encoded_data, float* data) {
    if (encoded_data[0] == 0xFF) {  
        for (int i = 0; i < MOTOR_NUM; i++) {
            int motor_num = (encoded_data[i*3+1] & 0xC0) >> 6;  
            if (i == motor_num) {
                int sign = (encoded_data[i*3+1] & 0x20) ? 1 : -1;  
                int value = ((encoded_data[i*3+1] & 0x1F) << 12) | (encoded_data[i*3+2] << 4) | (encoded_data[i*3+3] >> 4);  
                data[i] = sign * float(value) / 100;
            }
        }
    }
}

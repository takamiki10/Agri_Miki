#include "FloatArrayEncoder.h"
FloatArrayEncoder::FloatArrayEncoder() {
}

void FloatArrayEncoder::encode(float* float_data, uint8_t* encoded_data) {
    encoded_data[0] = 0xFF; 
    for(int i = 0; i < MOTOR_NUM; i++) {
        int data = int(float_data[i] * 100);  
        int sign = (data >= 0) ? 1 : 0;  
        int value = abs(data); 
        encoded_data[i*2+1] = (i << 6) | (sign << 5) | ((value >> 8) & 0x1F); 
        encoded_data[i*2+2] = value & 0xFF;  
    }
}

void FloatArrayEncoder::decode(uint8_t* encoded_data, float* data) {
    if(encoded_data[0] == 0xFF) {  
        for(int i = 0; i < MOTOR_NUM; i++) {
            int motor_num = (encoded_data[i*2+1] & 0xC0) >> 6;  
            if(i == motor_num) {
                int sign = (encoded_data[i*2+1] & 0x20) ? 1 : -1;
                int value = ((encoded_data[i*2+1] & 0x1F) << 8) | encoded_data[i*2+2]; 
                data[i] = sign * float(value) / 100;  
            }
        }
    }
}

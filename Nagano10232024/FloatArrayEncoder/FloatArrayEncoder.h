#ifndef FLOATARRAYENCODER_H
#define FLOATARRAYENCODER_H

#include <stdint.h>

#define MOTOR_NUM 3  // Define the number of motors (float values)

class FloatArrayEncoder {
public:
    FloatArrayEncoder();  // Constructor
    void encode(float* float_data, uint8_t* encoded_data);  // Method to encode float array
    void decode(uint8_t* encoded_data, float* data);  // Method to decode byte array to float
};

#endif  // FLOATARRAYENCODER_H

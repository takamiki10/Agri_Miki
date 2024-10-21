#ifndef FLOAT_ARRAY_ENCODER_H
#define FLOAT_ARRAY_ENCODER_H

#include <Arduino.h>

#define MOTOR_NUM 3  // We are working with 3 floats

class FloatArrayEncoder {
public:
    // Constructor
    FloatArrayEncoder();

    // Encode 3 float values into a byte array
    void encode(float* float_data, uint8_t* encoded_data);

    // Decode a byte array into 3 float values
    void decode(uint8_t* encoded_data, float* data);
};

#endif // FLOAT_ARRAY_ENCODER_H

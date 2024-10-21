#ifndef FLOAT_ARRAY_ENCODER_H
#define FLOAT_ARRAY_ENCODER_H

#include <Arduino.h>

#define MOTOR_NUM 3

class FloatArrayEncoder {
public:
    FloatArrayEncoder();
    void encode(float* float_data, uint8_t* encoded_data);
    void decode(uint8_t* encoded_data, float* data);
};

#endif

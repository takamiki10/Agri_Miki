#ifndef DYNAMIXEL_MOBILITY_H
#define DYNAMIXEL_MOBILITY_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
#define ADDR_TORQUE_ENABLE      64
#define ADDR_OPERATING_MODE     11

// Protocol version
#define PROTOCOL_VERSION        2.0

// Default setting
#define BAUDRATE                1000000
#define DEVICENAME              "/dev/ttyUSB0"
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

class DynamixelMobility {
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    int dxl_ids[4];

public:
    DynamixelMobility();
    bool init(int id1, int id2, int id3, int id4);
};

#endif // DYNAMIXEL_MOBILITY_H

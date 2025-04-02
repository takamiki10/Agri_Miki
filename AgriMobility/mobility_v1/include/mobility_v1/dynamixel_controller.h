#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <stdio.h>
#include <vector>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table addresses
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_POSITION      116
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_POSITION   132
#define ADDR_OPERATING_MODE     11

// Protocol version
#define PROTOCOL_VERSION        2.0

// Default settings
#define BAUDRATE                1000000
#define DEVICENAME              "/dev/ttyUSB0"
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define VELOCITY_CONTROL        1
#define POSITION_CONTROL        3

class DynamixelController {
public:
    DynamixelController(std::vector<int> ids);
    ~DynamixelController();

    bool initialize();
    void disableTorque();
    void enableTorque();
    void setOperationMode(int mode);
    void setVelocity(int id, int goalVelocity);
    void closePort();

private:
    std::vector<int> dynamixelIDs;
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
};

#endif // DYNAMIXEL_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk_examples/GetPosition.h"
// Control table address
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132
#define ADDR_OPERATING_MODE     11
// Protocol version
#define PROTOCOL_VERSION        2.0
// Default setting
#define BAUDRATE                1000000
#define DEVICENAME              "/dev/ttyUSB0"
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define DXL_MIN_POSITION_VALUE  0
#define DXL_MAX_POSITION_VALUE  4095
std::map<int, int> current_positions;
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
void set_goal_position(int id, int position) {
    uint8_t dxl_error;
    int32_t dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        current_positions[id] = position;
        ROS_INFO("Set Goal Position of ID %d = %d", id, position);
    } else {
        ROS_ERROR("Failed to set goal position of ID %d", id);
    }
}

void get_present_position(int id) {
    uint8_t dxl_error;
    uint32_t position;
    int32_t dxl_present_position = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, &position, &dxl_error);
    if (dxl_present_position != -1) {
        current_positions[id] = dxl_present_position;
        ROS_INFO("Present Position of ID %d = %d", id, dxl_present_position);
    } else {
        ROS_ERROR("Failed to get present position of ID %d", id);
    }
}
void disableTorqueControl(){
    for (int dxl_id = 0; dxl_id <= 4; dxl_id++) { 
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            continue; 
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            continue; 
        } else {
            ROS_INFO("Disabled TorqueControl of ID %d = %d", dxl_id, dxl_comm_result);
        }
    }
}

void chooseOperationType(int operationIndex){
    for(int dxl_id = 0; dxl_id <= 4; dxl_id++){
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, operationIndex, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            ROS_ERROR("Failed to set velocity control mode for DYNAMIXEL ID %d", dxl_id);
            continue;
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            ROS_ERROR("Failed to set velocity control mode for DYNAMIXEL ID %d", dxl_id);
            continue; 
        } else {
            if(operationIndex == 3){
                ROS_INFO("Operating Mode : Velocity Control Mode");
            }
            else if (operationIndex == 1){
                ROS_INFO("Operating Mode : Position Control Mode");
            }
        }
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "read_write_cpp_node");
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port");
        return 1;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to change the baudrate");
        return 1;
    }
    disableTorqueControl();
    chooseOperationType(4);
    for (int dxl_id = 0; dxl_id <= 4; dxl_id++) {
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            ROS_ERROR("Failed to connect DYNAMIXEL ID %d", dxl_id);
            continue; // Skip to the next iteration of the loop
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            ROS_ERROR("Failed to connect DYNAMIXEL ID %d", dxl_id);
            continue; // Skip to the next iteration of the loop
        } else {
            ROS_INFO("DYNAMIXEL ID %d has been successfully connected", dxl_id);
        }
        current_positions[dxl_id] = 0;
    }

    ROS_INFO("Ready to get & set Position.");
    while (ros::ok()) {
        int id, position;
        std::cout << "Enter Dynamixel ID (0 through 4): ";
        std::cin >> id;
        if (id < 0 || id > 5) {
            ROS_ERROR("Invalid ID. Please enter an ID from 1 through 5.");
            continue;
        }
        std::cout << "Enter desired position for Dynamixel (in degrees): ";
        std::cin >> position;
        int dxl_position = (position / 360.0) * (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) + DXL_MIN_POSITION_VALUE;
        set_goal_position(id, dxl_position);
        get_present_position(id);
    }
    portHandler->closePort();
    return 0;
}

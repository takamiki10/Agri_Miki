#include <dynamixel_sdk/dynamixel_sdk.h>
#include "mobility_v1/dynamixel_controller.h"


DynamixelController::DynamixelController(std::vector<int> ids) 
    : dynamixelIDs(ids) {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}
DynamixelController::~DynamixelController() {
    closePort();
}


bool DynamixelController::initialize() {
    if (!portHandler->openPort()) {
        printf("Failed to open the port\n");
        return false;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        printf("Failed to change the baudrate\n");
        return false;
    }

    /* disableTorque();
    setOperationMode(VELOCITY_CONTROL);
    enableTorque(); */

    return true;
}

void DynamixelController::disableTorque() {
    for (int dxl_id : dynamixelIDs) {
        uint8_t dxl_error;
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to disable torque for ID %d: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Disabled torque for ID %d\n", dxl_id);
        }
    }
}

void DynamixelController::enableTorque() {
    for (int dxl_id : dynamixelIDs) {
        uint8_t dxl_error;
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to enable torque for ID %d: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Enabled torque for ID %d\n", dxl_id);
        }
    }
}

void DynamixelController::setOperationMode(int mode) {
    disableTorque();
    for (int dxl_id : dynamixelIDs) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, mode, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to set operation mode for ID %d: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            printf("Set operation mode for ID %d to %d\n", dxl_id, mode);
        }
    }
    enableTorque();
}

void DynamixelController::setVelocity(int id, int goalVelocity) {
    uint8_t dxl_error;
    int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, goalVelocity, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
        printf("Set velocity for ID %d = %d\n", id, goalVelocity);
    } else {
        printf("Failed to set goal velocity for ID %d\n", id);
    }
}

void DynamixelController::closePort() {
    portHandler->closePort();
    printf("Closed the Dynamixel port.\n");
}

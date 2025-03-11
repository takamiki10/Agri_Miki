#include "dynamixelMobility.h"

DynamixelMobility::DynamixelMobility() {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

bool DynamixelMobility::init(int id1, int id2, int id3, int id4) {
    dxl_ids[0] = id1;
    dxl_ids[1] = id2;
    dxl_ids[2] = id3;
    dxl_ids[3] = id4;

    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port");
        return false;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to change the baudrate");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_OPERATING_MODE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        ROS_INFO("DYNAMIXEL ID %d initialized successfully", dxl_ids[i]);
    }

    ROS_INFO("All DYNAMIXEL motors initialized successfully.");
    return true;
}

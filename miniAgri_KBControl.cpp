#include <iostream>
#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_POSITION   132
#define ADDR_OPERATING_MODE     12 // Update the address for operating mode

// Protocol version
#define PROTOCOL_VERSION        2.0

// Default setting
#define BAUDRATE                1000000
#define DEVICENAME              "/dev/ttyUSB0"
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define DXL_MIN_VELOCITY_VALUE  -1000
#define DXL_MAX_VELOCITY_VALUE  1000

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

void set_goal_velocity(int id, int velocity) {
    uint8_t dxl_error;
    int32_t dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else {
        ROS_ERROR("Failed to set goal velocity of ID %d", id);
    }
}

void stop_motor(int id) {
    set_goal_velocity(id, 0);
}

void disableTorqueControl(){
    for (int dxl_id = 0; dxl_id <= 4; dxl_id++) { // Assuming you have 3 motors
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            //ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the function with error status
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            //ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the function with error status
        } else {
            //ROS_INFO("DYNAMIXEL ID %d has been successfully connected", dxl_id);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_control_cpp_node");
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port");
        return 1; // Exit the program with error status
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to change the baudrate");
        return 1; // Exit the program with error status
    }
    disableTorqueControl();
    // Enable Velocity Control
    for(int dxl_id = 0; dxl_id <= 4; dxl_id++){
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            ROS_ERROR("Failed to set velocity control mode for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the program with error status
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            ROS_ERROR("Failed to set velocity control mode for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the program with error status
        } else {
            ROS_INFO("Operating Mode : Velocity Control Mode");
        }
    }

    // Enable Torque Control
    for (int dxl_id = 0; dxl_id <= 4; dxl_id++) { // Assuming you have 3 motors
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the program with error status
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; // Exit the program with error status
        } else {
            ROS_INFO("DYNAMIXEL ID %d has been successfully connected", dxl_id);
        }
    }

    ROS_INFO("Ready for velocity control.");

    while (ros::ok()) {
        char input;
        std::cout << "Press 'c' to move clockwise, 's' to stop, 'a' to move counterclockwise, or 'q' to quit: ";
        std::cin >> input;

        switch(input) {
            case 'q':
                set_goal_velocity(0, DXL_MAX_VELOCITY_VALUE); // Assuming motor ID 1 is for clockwise movement
                break;
            case 'a':
                stop_motor(0); // Assuming motor ID 1 is stopped
                break;
            case 'z':
                set_goal_velocity(0, DXL_MIN_VELOCITY_VALUE); // Assuming motor ID 1 is for counterclockwise movement
                break;
            case 'w':
                set_goal_velocity(1, DXL_MAX_VELOCITY_VALUE); // Assuming motor ID 2 is for clockwise movement
                break;
            case 's':
                stop_motor(1); // Assuming motor ID 2 is stopped
                break;
            case 'x':
                set_goal_velocity(1, DXL_MIN_VELOCITY_VALUE); // Assuming motor ID 2 is for counterclockwise movement
                break;
            case 'e':
                set_goal_velocity(2, DXL_MAX_VELOCITY_VALUE); // Assuming motor ID 3 is for clockwise movement
                break;
            case 'd':
                stop_motor(2); // Assuming motor ID 3 is stopped
                break;
            case 'c':
                set_goal_velocity(2, DXL_MIN_VELOCITY_VALUE); // Assuming motor ID 3 is for counterclockwise movement
                break;
            case 'r':
                set_goal_velocity(3, DXL_MAX_VELOCITY_VALUE); // Assuming motor ID 2 is for clockwise movement
                break;
            case 'f':
                stop_motor(3); // Assuming motor ID 2 is stopped
                break;
            case 'v':
                set_goal_velocity(3, DXL_MIN_VELOCITY_VALUE); // Assuming motor ID 2 is for counterclockwise movement
                break;
            case 't':
                set_goal_velocity(4, DXL_MAX_VELOCITY_VALUE); // Assuming motor ID 3 is for clockwise movement
                break;
            case 'g':
                stop_motor(4); // Assuming motor ID 3 is stopped
                break;
            case 'b':
                set_goal_velocity(4, DXL_MIN_VELOCITY_VALUE); // Assuming motor ID 3 is for counterclockwise movement
                break;
            case 'p':
                disableTorqueControl();
                return 0;
            default:
                ROS_ERROR("Invalid input.");
        }
    }

    return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "sensor_msgs/Joy.h"

// Control table address
#define ADDR_OPERATING_MODE     11      //1 is Velocity Control, 3 is Position Control Mode, 4 is Extended Position Control(Multiturn)
#define ADDR_VELOCITY_LIMIT     44      
#define ADDR_TORQUE_ENABLE      64      
#define ADDR_GOAL_VELOCITY      104     
#define ADDR_PRESENT_VELOCITY   128
#define ADDR_PRESENT_POSITION   132   
  
#define MODE_VELOCITY           1
#define MODE_POSITION           3
#define MODE_MULTITURN          4

// Protocol version
#define PROTOCOL_VERSION        2.0

// Default setting
#define BAUDRATE                1000000
#define DEVICENAME              "/dev/ttyUSB0"
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define DXL_MIN_VELOCITY_VALUE  -400
#define DXL_MAX_VELOCITY_VALUE  400

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

int chosen_Dynamixel_ID;
sensor_msgs::Joy prev_joy_state;


void set_goal_velocity(int id, int velocity) {
    uint8_t dxl_error;
    int32_t dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        //ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else {
        ROS_ERROR("Failed to set goal velocity of ID %d", id);
    }
}

void stop_motor(int id) {
    set_goal_velocity(id, 0);
}

void disableTorqueControl(){
    for (int dxl_id = 2; dxl_id <= 6; dxl_id++) { 
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            continue; 
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            continue; 
        }
    }
}

void enableTorqueControl(){
    for (int dxl_id = 2; dxl_id <= 6; dxl_id++) { 
        uint8_t dxl_error;
        int32_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
            ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; 
        } else if (dxl_error != 0) {
            ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
            ROS_ERROR("Failed to enable torque control for DYNAMIXEL ID %d", dxl_id);
            continue; 
        }
    }
}

void printCurrentSpeed() {
    std::string velocities;

    for (int dxl_id = 2; dxl_id <= 6; dxl_id++) { 
        uint32_t dxl_present_velocity = 0; // Initialize velocity variable
        int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VELOCITY, &dxl_present_velocity, nullptr);
        
        if (dxl_comm_result == COMM_SUCCESS) {
            if (!velocities.empty()) {
                velocities += ",";
            }
            velocities += std::to_string(dxl_present_velocity);
        } else {
            ROS_ERROR("Failed to read velocity for DYNAMIXEL ID %d", dxl_id);
        }
    }

    ROS_INFO("[%s]", velocities.c_str());
}



void chooseOperationType(int operationIndex){
    for(int dxl_id = 2; dxl_id <= 6; dxl_id++){
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
            ROS_INFO("Operating Mode : Velocity Control Mode");
        }
    }
}

void messageCB(const sensor_msgs::Joy::ConstPtr& msg) {
    for (size_t i = 0; i <= 3; ++i) {
        if (msg->buttons[i] == 1) {
            ROS_INFO("check2");
            switch (i){
                case 0:
                    chosen_Dynamixel_ID = 2;
                    break;
                case 1:
                    chosen_Dynamixel_ID = 3;
                    break;
                case 2:
                    chosen_Dynamixel_ID = 4;
                    break;
                case 3:
                    chosen_Dynamixel_ID = 5;
                    break;
                default:
                    break;
            }
        }
    }

    set_goal_velocity(chosen_Dynamixel_ID, msg->axes[1] * DXL_MAX_VELOCITY_VALUE);
    printCurrentSpeed();
    prev_joy_state = *msg; // Update the previous joystick state
}


int main(int argc, char **argv) {
    chosen_Dynamixel_ID = 0;
    
    ros::init(argc, argv, "velocity_control_cpp_node");
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
    chooseOperationType(MODE_VELOCITY);
    enableTorqueControl();

    ROS_INFO("Ready for velocity control.");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, messageCB);

    // Spin
    ros::spin();
    return 0;
}

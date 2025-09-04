#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "dynamixel_controller.h"

//---------------DYNAMIXEL CONFIG---------------//
const std::vector<int> dynamixelIDs = {11, 12, 13, 14};
const int AXES_1_MOTORS[] = {11, 12};  // Controlled by axes[1]
const int AXES_2_MOTORS[] = {13, 14};  // Controlled by axes[2] (inverted)

const int MAX_VELOCITY = 100;  // Adjust as needed (rev/min)

//----------------------------------------------//

DynamixelController controller(dynamixelIDs);

// Callback function for joystick input
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    if (msg->axes.size() < 3) {
        ROS_WARN_THROTTLE(1, "Joy message does not have enough axes.");
        return;
    }

    double axis1_value = msg->axes[1];  // Forward/Backward
    double axis2_value = msg->axes[4];  // Inverted Forward/Backward

    int velocity1 = static_cast<int>(axis1_value * MAX_VELOCITY);
    int velocity2 = static_cast<int>(-axis2_value * MAX_VELOCITY);  // Inverted for motors 13 & 14

    // Set velocity for Dynamixel 11 and 12
    for (int id : AXES_1_MOTORS) {
        controller.setVelocity(id, velocity1);
    }

    // Set velocity for Dynamixel 13 and 14 (reversed)
    for (int id : AXES_2_MOTORS) {
        controller.setVelocity(id, velocity2);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamixel_joy_control");
    ros::NodeHandle nh;

    // Initialize Dynamixel Controller
    if (!controller.initialize()) {
        ROS_ERROR("Failed to initialize Dynamixel controller.");
        return 1;
    }

    controller.setOperationMode(VELOCITY_CONTROL);
    ROS_INFO("Dynamixel velocity control initialized. Listening to /joy.");

    // Subscribe to joystick topic
    ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);

    // Keep ROS running
    ros::spin();

    return 0;
}

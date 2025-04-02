#include <stdio.h>
#include <ros/ros.h>
#include "dynamixel_controller.h"

//---------------ENTER DYNAMIXEL IDS HERE!---------------//

const std::vector<int> dynamixelIDs = {11, 12, 13, 14};

//------------------------------------------------------//

DynamixelController controller(dynamixelIDs);

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_write_cpp_node");

    if (!controller.initialize()) {
        return 1;
    }
    controller.setOperationMode(VELOCITY_CONTROL);


    ROS_INFO("Ready to get & set Velocity");

    while (ros::ok()) {
        int id, velocity;
        printf("Enter Dynamixel ID (choose from: ");
        for (size_t i = 0; i < dynamixelIDs.size(); i++) {
            printf("%d", dynamixelIDs[i]);
            if (i < dynamixelIDs.size() - 1) {
                printf(", ");
            }
        }
        printf("): ");
        scanf("%d", &id);

        // Validate input ID
        bool valid_id = false;
        for (int valid : dynamixelIDs) {
            if (valid == id) {
                valid_id = true;
                break;
            }
        }

        if (!valid_id) {
            printf("Invalid ID. Please enter a valid Dynamixel ID.\n");
            continue;
        }

        printf("Enter desired speed for Dynamixel (in rev/min): ");
        scanf("%d", &velocity);

        // Set the velocity
        controller.setVelocity(id, velocity);
    }

    return 0;
}

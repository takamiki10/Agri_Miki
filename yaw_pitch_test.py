#!/usr/bin/env python3
import os
import rospy
from dynamixel_sdk import *

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 1000000            # Dynamixel default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MAXIMUM_POSITION_VALUE  = 4095              # Maximum position value

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

pos1 = 0
pos2 = 0

def set_goal_pos_callback(data):
    global pos1, pos2
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    if data.id == 1:
        pos1 += data.position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, pos1 % (DXL_MAXIMUM_POSITION_VALUE + 1))
    elif data.id == 2:
        pos2 += data.position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, pos2 % (DXL_MAXIMUM_POSITION_VALUE + 1))
    else:
        print("Invalid Dynamixel ID:", data.id)

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position

def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    rospy.spin()

def main():
    global pos1, pos2

    # Initialize positions of IDs 1 and 2 to 0
    try:
        portHandler.openPort()
        print("Succeeded to open the port")

        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("Failed to initialize position for ID 1: %s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Failed to initialize position for ID 1: %s" % packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("Failed to initialize position for ID 2: %s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Failed to initialize position for ID 2: %s" % packetHandler.getRxPacketError(dxl_error))

        print("Ready to get & set Position.")

        while True:
            try:
                DXL_ID = int(input("Enter Dynamixel ID: "))
                position = int(input("Enter desired position for Dynamixel: "))
                # Update total rotations
                if DXL_ID == 1:
                    pos1 += position
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pos1 % (DXL_MAXIMUM_POSITION_VALUE + 1))
                elif DXL_ID == 2:
                    pos2 += position
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pos2 % (DXL_MAXIMUM_POSITION_VALUE + 1))
                else:
                    print("Invalid Dynamixel ID:", DXL_ID)
                    break

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                while True:
                    try:
                        other_id = input("Enter other Dynamixel ID (press q to quit): ")
                        if other_id == 'q':
                            break
                        other_id = int(other_id)
                        other_position = int(input("Enter desired position for other Dynamixel: "))
                        # Update total rotations
                        if other_id == 1:
                            pos1 += other_position
                            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, other_id, ADDR_GOAL_POSITION, pos1 % (DXL_MAXIMUM_POSITION_VALUE + 1))
                        elif other_id == 2:
                            pos2 += other_position
                            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, other_id, ADDR_GOAL_POSITION, pos2 % (DXL_MAXIMUM_POSITION_VALUE + 1))
                        else:
                            print("Invalid Dynamixel ID:", other_id)
                            break
                        
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s" % packetHandler.getRxPacketError(dxl_error))
                    except KeyboardInterrupt:
                        print("\nExiting...")
                        break
            except KeyboardInterrupt:
                print("\nExiting...")
                break

    except Exception as e:
        print("An error occurred:", e)
    finally:
        portHandler.closePort()


if __name__ == '__main__':
    main()

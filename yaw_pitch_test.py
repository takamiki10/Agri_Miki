#!/usr/bin/env python3
import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
# Import for Windows or Linux
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
# Control table address
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
# Protocol version
PROTOCOL_VERSION        = 2.0
# Default setting
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
DXL_MIN_POSITION_VALUE  = 0
DXL_MAX_POSITION_VALUE  = 4095
current_positions = {}
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
def set_goal_pos_callback(data):
    global current_positions
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)
    if dxl_comm_result == COMM_SUCCESS:
        current_positions[data.id] = data.position
def get_present_pos(req):
    global current_positions
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    if dxl_comm_result == COMM_SUCCESS:
        current_positions[req.id] = dxl_present_position
        print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position
def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    rospy.spin()
def main():
    global DEVICENAME
    global current_positions
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    print("Ready to get & set Position.")

    for dxl_id in range(1, 4):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print("DYNAMIXEL ID %d has been successfully connected" % dxl_id)
        current_positions[dxl_id] = 0
    while True:
        try:
            dxl_id = int(input("Enter Dynamixel ID: "))
            if dxl_id not in current_positions:
                print("Invalid ID. Please enter a valid ID.")
                continue
            position = int(input("Enter desired position for Dynamixel (in degrees): "))
            dxl_position = int((position / 360) * (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) + DXL_MIN_POSITION_VALUE)
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_position)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
        except ValueError:
            print("Invalid input. Please enter a valid ID and position.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break
if __name__ == '__main__':
    main()

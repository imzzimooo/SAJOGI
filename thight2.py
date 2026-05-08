import os
import time
from dynamixel_sdk import *
DEVICENAME = 'COM10'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
thighs=[10,4,1,7] #허벅지 모터 아이디
knees=[9,3,6,0] #무릎모터 아이디
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
thigh_target_points = [1300, 1400, 1500, 1600, 1700] #허벅지 1300 -> 1700 100 간격 회전
knee_target_points=[1200, 1150, 1100, 1050] # 무릎 1200->1050 50간격 회전

try:
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    motors=[0,1,2,3,4,5,6,7,8,9,10,11] #토크 켜기
    for id in motors:
        packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, 1)

    # 허벅지 명령
    for target in thigh_target_points:
        for id in thighs:
            packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, target)
        time.sleep(0.5)
    #무릎 명령
    for target in knee_target_points:
        for id in knees:
            packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, target)
        time.sleep(0.5)

finally:
    #packetHandler.write1ByteTxRx(portHandler, DXL_RF_ID, ADDR_PRO_TORQUE_ENABLE, 0)
    #packetHandler.write1ByteTxRx(portHandler, DXL_LF_ID, ADDR_PRO_TORQUE_ENABLE, 0)
    #packetHandler.write1ByteTxRx(portHandler, DXL_RB_ID, ADDR_PRO_TORQUE_ENABLE, 0)
    #packetHandler.write1ByteTxRx(portHandler, DXL_LB_ID, ADDR_PRO_TORQUE_ENABLE, 0)
    portHandler.closePort()

import os
import time
import math
from dynamixel_sdk import *

DEVICENAME = '/dev/tty.usbserial-FT3WKKNI'  #Devidename needs to be changed depending on user
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

thighs = [10, 4, 1, 7]
knees  = [9, 3, 6, 0]

ADDR_PRO_TORQUE_ENABLE   = 64
ADDR_PRO_GOAL_POSITION   = 116
ADDR_PRO_PRESENT_POSITION = 132

# ── Position ranges ────────────────────────────────────────────────
THIGH_DOWN = 1300
THIGH_UP   = 1700

KNEE_DOWN  = 1200
KNEE_UP    = 1050   # "up" means more bent (smaller value)

# ── Trajectory settings ────────────────────────────────────────────
STEPS        = 60    # number of interpolation steps per phase
STEP_DELAY   = 0.02  # seconds between steps → total ~1.2 s per phase

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def ease_in_out(t: float) -> float:
    """Smoothstep easing: slow start, fast middle, slow end (t in [0,1])."""
    return t * t * (3 - 2 * t)


def interpolate(start: int, end: int, t: float) -> int:
    """Linear interpolation with smoothstep easing applied to t."""
    eased = ease_in_out(t)
    return int(start + (end - start) * eased)


def move_phase(thigh_start, thigh_end, knee_start, knee_end, steps=STEPS, delay=STEP_DELAY):
    """Smoothly move thighs and knees together from start to end positions."""
    for step in range(steps + 1):
        t = step / steps
        thigh_pos = interpolate(thigh_start, thigh_end, t)
        knee_pos  = interpolate(knee_start,  knee_end,  t)

        for id in thighs:
            packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, thigh_pos)
        for id in knees:
            packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, knee_pos)

        time.sleep(delay)


def enable_torque(motor_ids):
    for id in motor_ids:
        packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, 1)


def disable_torque(motor_ids):
    for id in motor_ids:
        packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, 0)


try:
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    all_motors = thighs + knees
    enable_torque(all_motors)

    print("Phase 1: DOWN → UP")
    move_phase(THIGH_DOWN, THIGH_UP, KNEE_DOWN, KNEE_UP)

    time.sleep(0.3)  # brief pause at top

    print("Phase 2: UP → DOWN")
    move_phase(THIGH_UP, THIGH_DOWN, KNEE_UP, KNEE_DOWN)

    print("Done.")

finally:
    disable_torque(all_motors)
    portHandler.closePort()

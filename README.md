SAJOGI Ver.1 — Spot-Micro Inspired Quadruped Robot

📌 Overview

SAJOGI Ver.1 is a custom-built quadruped robot inspired by the Spot Micro open-source project.

The current iteration utilizes 12 high-torque servo motors, a Raspberry Pi 4B as the main controller, and an Adafruit PCA9685 PWM driver. The servo driver has been upgraded with a 1200 μF capacitor to ensure stable and consistent power delivery for precise servo control.

SAJOGI currently has basic capabilities, including forward walking and crouching. We are actively working on LiDAR integration (YD LiDAR G4) and simultaneous localization and mapping (SLAM) experiments.

Note on Limitations: Due to unexpected interference caused by the selected servo motor size, the existing chassis design presents several structural limitations. A complete chassis redesign is planned to significantly improve the robot's durability, stability, and overall performance.


12 high-torque servo motors for dynamic motion.

Raspberry Pi 4B utilized as the primary embedded controller.

Adafruit PCA9685 PWM driver enhanced with a 1200 μF capacitor for stable power.

Implemented gait patterns: Forward walking & crouching.

LiDAR integration (YD LiDAR G4) — In Progress

SLAM experimentation — In Progress

Custom 3D-printed frame adapted from the KDY0523 Spot Micro CAD model.

🔧 Hardware Specifications

Component

Description

Controller

Raspberry Pi 4B

Servo Driver

Adafruit PCA9685 + 1200 μF capacitor

Servo Motors

DM-CLS400MD × 12 (High-Torque)

Power Supply

2-cell 1400 mAh LiPo battery

Frame

Modified KDY0523 Spot Micro STLs

Sensors

YD LiDAR G4 (planned), IMU (planned)

🚀 Getting Started (Planned)

Details on the software stack, required libraries, and calibration steps will be provided here once the chassis redesign is complete and the software becomes stable.

📁 References

Helpful resources and inspiration for this project are organized below:

SpotMicroAI Open Source: https://gitlab.com/public-open-source/spotmicroai

Adafruit PCA9685 Guide: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

Spot Micro CAD (Thingiverse): https://www.thingiverse.com/thing:3445283

TurtleBot3 SBC Setup: https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

YOLOv5 on Raspberry Pi 4: https://github.com/jordan-johnston271/yolov5-on-rpi4-2020

Spot ROS Documentation: http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/

Spot Micro Walking Demo (Video): https://www.youtube.com/watch?v=FM3FzZ81KOU

OpenQuadruped (Spot Mini Mini): https://github.com/OpenQuadruped/spot_mini_mini

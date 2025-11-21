# SAJOGI

SAJOGI Ver.1 — A Spot-Micro Inspired Quadruped Robot

📌 Overview

SAJOGI Ver.1 is a custom-built quadruped robot inspired by Spot Micro.
It uses 12 high-torque servo motors, a Raspberry Pi 4B, and a PCA9685 PWM driver with a 1200 μF stabilized capacitor to ensure smooth operation.
SAJOGI is capable of walking forward and crouching, and is currently undergoing LiDAR integration and SLAM experiments.
Despite its basic mobility being functional, the robot has some structural limitations due to unexpected motor size mismatches. A redesign is planned to increase durability and performance.

* 12 high-torque servo motors
* Raspberry Pi 4B as the main controller
* PCA9685 servo driver with added 1200 μF capacitor
* Forward walking & crouching implemented
* LiDAR integration (YD LiDAR G4) — in progress
* SLAM experimentation — in progress
* Custom frame modeled after KDY0523 Spot Micro


🔧 Hardware Specifications

| Component    | Description                          |
| ------------ | ------------------------------------ |
| Controller   | Raspberry Pi 4B                      |
| Servo Driver | Adafruit PCA9685 + 1200 μF capacitor |
| Servo Motors | DM-CLS400MD x12                      |
| Power        | 2-cell 1400 mAh LiPo battery         |
| Frame        | Modified KDY0523 Spot Micro STL      |
| Sensors      | YD LiDAR G4 (planned), IMU (planned) |


📁 reference
https://gitlab.com/public-open-source/spotmicroai
https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
https://www.thingiverse.com/thing:3445283
https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup
https://github.com/jordan-johnston271/yolov5-on-rpi4-2020
http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/
https://www.youtube.com/watch?v=FM3FzZ81KOU
https://github.com/OpenQuadruped/spot_mini_mini

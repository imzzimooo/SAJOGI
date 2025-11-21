SAJOGI Ver.1 — Spot-Micro Inspired Quadruped Robot

📌 Overview
SAJOGI Ver.1 is a custom-built quadruped robot inspired by the open-source Spot Micro project.
It is powered by 12 high-torque servo motors, controlled by a Raspberry Pi 4B and a PCA9685 PWM driver. To ensure stable servo control, the power system has been upgraded with a 1200 μF capacitor. Currently, SAJOGI is capable of basic locomotion such as forward walking and crouching.
The project is actively evolving, with ongoing experiments in LiDAR integration and SLAM (Simultaneous Localization and Mapping).

Actuation: 12 × High-torque servo motors for 3-DOF per leg.
Controller: Raspberry Pi 4B as the main computing unit.
Power Management: PCA9685 servo driver stabilized with a 1200 μF capacitor.
Movement: Implemented forward walking gait and crouching mechanics.
Sensing (In Progress): LiDAR integration using YD LiDAR G4.
Navigation (In Progress): SLAM experimentation.
Design: Custom frame modeled after KDY0523 Spot Micro.


🔧 Hardware Specifications
* Description
* Raspberry Pi 4B
* Adafruit PCA9685 + 1200 μF Capacitor
* DM-CLS400MD Servo Motor × 12
* 2-cell 1400 mAh LiPo Battery
*  Modified KDY0523 Spot Micro STL
* YD LiDAR G4 (Planned), IMU (Planned)

  graph LR
    %% 전원부
    LiPo[LiPo Battery 7.4V] --> UBEC_Motor[고용량 UBEC\n(7.4V -> 6V, 15A+)]
    LiPo --> UBEC_RPi[별도 UBEC\n(7.4V -> 5V, 3A+)]

    %% 라즈베리파이
    UBEC_RPi -->|5V / GND| RPi[Raspberry Pi 4B]
    RPi -->|USB| Cam[Camera]

    %% 모터 제어부
    UBEC_Motor -->|6V / GND| PCA[PCA9685 Driver]
    RPi -->|I2C (SDA/SCL)| PCA
    RPi -->|3.3V / GND| PCA
    
    %% 서보모터
    PCA -->|PWM / Power| Servo1[Servo x12]
    Cap[Capacitor 1200uF] -.- PCA

    %% LCD (전압 주의)
    RPi -->|I2C (SDA/SCL)| LCD[I2C LCD]
    RPi -->|5V / GND| LCD
  

📁 References
* https://gitlab.com/public-open-source/spotmicroai
* https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
* https://www.thingiverse.com/thing:3445283
* https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup
* https://github.com/jordan-johnston271/yolov5-on-rpi4-2020
* http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/
* https://www.youtube.com/watch?v=FM3FzZ81KOU
* https://github.com/OpenQuadruped/spot_mini_mini)

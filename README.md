# MicroMouse
This repository consists code, circuit diagrams, images and lists out components used in this project. 

This project used an ESP32 and the code for it is in the ino file.


**Components used:**
1) ESP32
2) VL53L0X Lidar Sensors
3) MPU6500 Gyroscope
4) 9V Battery Pack
5) N20 Motors with DRV8833 Driver
6) N20 Wheels

**How it works:**
When the Micromouse solves a maze, it continuously senses openings in its surroundings using Lidar sensors and when it finds an opening, it turns either right or left based on the conditions presented in the maze, helping us solve the maze efficiently.

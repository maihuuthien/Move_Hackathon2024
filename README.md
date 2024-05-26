# #Move-Hackathon2024
Before run Framework, sure that you installed ROS1 and Carla.

# Initial
Step1: Build ros package 02_RosCommunication -> source to this package

Step2: Build ros package 01_RosBridge -> source to this package.

For Carla has different version (not 0.9.13), you must change this in 01_RosBridge/src/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION file. Then build again.

## Start:
Launch Carla server.

Make sure you source to ros package at step 2 in all Ros Terminal.

Ros Terminal 1 (Ros Bridge):
    
    roslaunch carla_ros_bridge carla_ros_bridge.launch

Ros Terminal 2 (Client):
    
    python /03_Client/main.py
    
Ros Terminal 3 (Dev code):
   
    python /04_Template/main.py

## Move API:

You can get template in /04_Template

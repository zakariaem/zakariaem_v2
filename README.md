# zakariaem_v2 
Introducing the Enhanced Electromyography and Hand Motion Description (zakariaem) Database Version 2.0

## Description

The zakariaem database contains sEMG (surface electromyography) signals and motion capture of human hand movements. The sEMG signals are captured using a Myo armband, while motion acquisition is achieved with a Leap Motion stereoscopic camera.

Currently, the zakariaem v2 provides ROS codes specifically designed for Windows to take advantage of manufacturer drivers. The new version incorporates the following improvements:

- The Leap Motion utilizes the 4.0.0 SDK. This upgraded C library (excluding C++) offers enhanced speed, robustness, and precision compared to the previous version.
- The Myo utilizes the 0.9.0 SDK, providing raw 200Hz EMG data (compared to 50Hz in the zakariaem_v1).

## How to use it

### ros_zakariaem

Once built, the ros_zakariaem package offers 3 nodes:

##### myo_subscriber_node
This node demonstrates how to subscribe to the Myo EMG data. It subscribes to the **myo_raw** topic and displays the EMG vector in the command prompt as it is sent through the topic.

##### Leap_subscriber_node
This node illustrates how to subscribe to the Leap Motion data. It subscribes to the **Leapmotion_raw** topic and displays the coordinate of the thumb metacarpal bone in the command prompt.

##### joint_coordinates_publisher
This code subscribes to the **leapmotion_raw** topic and publishes the corresponding joint coordinates in the **joint_coordinates_from_leap** topic. The header information is copied for future synchronization purposes.

### ros_leapmotion

Instructions for the ros_leapmotion package can be found here: https://github.com/SimonKirchhofer-UCA/ROS-LeapMotion-Windows

### ros_myo

Instructions for the ros_myo package can be found here : https://github.com/SimonKirchhofer-UCA/ROS_Myo_Windows



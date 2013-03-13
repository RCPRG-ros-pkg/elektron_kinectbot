display: Android Teleop
description: Drive a turtlebot from Android with a touch joystick and video feed.
platform: elektron
launch: elektron_teleop/android_teleop.launch
interface: elektron_teleop/android_teleop.interface
icon: elektron_teleop/icon.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel

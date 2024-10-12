# PointControlMiniplank WIP
This is a work in progress demonstratuion of a model railway point control system
As of 12/10 24 the version is "working" 
the main sketch is for an arduino fitted in a mimic switch controller, the slave alrduno is mounted under the layout to control 8 sensors
The final version will include multiple sensors and 32 points using PCA9685 servo PWM modules and WS2812b LED strips and light pipes to mimic display
features:
1. 1 long push enter **Caibrate point positions** ( select point position on switch, adjust etc. Short push undo and exit, long push save and exit)
2. 2 short pushes. Enable/Disable  **paired or grouped points** to operate together (see comments in code list )
3. 3 short pushes. Enable/Disable **Centralise Servo** (Select switch to move servo to mid position
4. 4 short pushes. enter **change move speed** (move encoder and operate point switch to change speed. all points throw in approx the same time irrespective of throw distance)
5. 6 short presses. **Reset panel**
Slave module (I2C) accepts 8 sensors. this is working for demo but needs modification for RS485 comms to JMRI using  C/MRI  

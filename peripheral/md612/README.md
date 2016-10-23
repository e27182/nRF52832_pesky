# Invensense Motion Driver 6.12 Example

It is an adaptation of example from Motion Driver SDK from Invensense.
It uses COM port to send data to PC, and on PC you should do:

python projects\external\motion_driver_6.12\eMPL-pythonclient\eMPL-client.py **COM6**

Where instead of COM6 put your COM port number. Example were tested on Python 2.7.11 from Anaconda 4.0.0 package.

These commands turn off individual sensors:
8 - toggle accelerometer on/off
9 - toggle gyroscope on/off
0 - toggle compass on/off

These commands send individual sensor data or fused data to the PC:
a - accelerometer
g - gyroscope
c - compass
e - Euler angles
r - rotation matrix
q - quaternion
i - linear acceleration (?)
o - gravity vector

Other commands:
w - prints status of the compass (?)
f - toggle DMP
m - test motion interrupt hardware feature
x - reset board
v - toggle 6x LP quaternion
d - if logging enabled, it dumps all registers
p - if DMP disabled - turns on **low power accel. mode**
t - run self test
, - set HW INT to gesture mode only
. - set HW INT to continuous mode
1-5 - changes data rate from gyro and accell
6 - toggle pedometer display
7 - reset pedometer

Note that when running python application, it will not react on keypresses when console window in focus.
You should focus windows with rotating 3D box and then keypresses will work.

[TODO: Add video]
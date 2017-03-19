# BLE: HID Joystick

![ble_app_hids_joystick_md612](https://cloud.githubusercontent.com/assets/14309815/24084556/fed7d74e-0cf4-11e7-8a1b-c86b68064d26.gif)

Adapted from ble_app_hids_mouse_md612 example.

After flashing you should see board as "Nordic_Joystick" with gamepad icon on your phone/tablet/PC.

Code contains same hardcoded calibration parameters as mouse example.
'Self-test' routine executes on startup to recalibrate accel and gyro, so, please, align board horizontally on startup to get correct calibration.
If you wish to calibrate magnetometer, you should do good '8' figure along with rotations on each axis (X, -X, Y, -Y, Z, -Z) and Invensense's implementation will update calibration for magnetometer automagically.

One thing that is not solved - GamePad API in web-browser can see connected joystick as '墥嫾㱳i (Vendor: 1915 Product: eeee)' with 6 buttons and 3 axes. And it reacts on taps, but axes are always zero.

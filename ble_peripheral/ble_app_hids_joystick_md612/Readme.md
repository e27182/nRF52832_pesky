# BLE: HID Joystick

![image](https://cloud.githubusercontent.com/assets/14309815/22438204/bef53b62-e733-11e6-9408-a8c41516bd13.gif)

![ble_app_hids_mouse_md612 buttons](https://cloud.githubusercontent.com/assets/14309815/22778568/16032188-eec0-11e6-97be-2b5ea431b305.gif)

Adapted ble_app_hids_mouse example from Nordic Semi SDK + adapted Motion Driver 6.12 from Invensense.

After flashing you should see board as "Nordic_Mouse" with mouse icon on your phone/tablet/PC.

Code contains hardcoded calibration parameters adapted to my environment:

```C
    long bias[3];
    bias[0] = 6211584;
    bias[1] = -2068480;
    bias[2] = -2611200;
    inv_set_accel_bias(bias, 3);
    bias[0] = -748747;
    bias[1] = 1786825;
    bias[2] = 1045736;
    inv_set_gyro_bias(bias, 3);
    bias[0] = -10261884;
    bias[1] = 63218332;
    bias[2] = -29328029;
    inv_set_compass_bias(bias, 3);
```

To get this values I extended MD612 example from this repository, so that when pressing 'b' - it prints all calculated bias values for each sensor.
Also 'self-test' routine executes on startup to recalibrate accel and gyro, so, please, align board horizontally on startup to get correct calibration.
If you wish to calibrate magnetometer, you should do good '8' figure along with rotations on each axis (X, -X, Y, -Y, Z, -Z) and Invensense's implementation will update calibration for magnetometer automagically.

Overall stability with calibrated sensors is very good, but sometimes I see a freeze for a second. As for now, I'm not sure what is the cause of it.

Part of the code taken from MD612 example is commented out as now we have stack overflow issues, so be careful when extending it.

# MPU9250

This example shows basic usage of MPU9250 with Invensense SDK (Motion Driver 6.12). It inits MPU9250, inits DMP, inits DMP data-ready interrupt and repetedly reads 6x low-power quaternion.

In **core** folder you can find adapted version of MD, which you can compare against **core_orig** for changes.
Also **inv_pesky.h** contains ported functions required by MD.

Compile, flash, open RTT Viewer and you should see repeating logs:

![image](https://cloud.githubusercontent.com/assets/14309815/18893064/ee61033a-8514-11e6-9834-d1f75523fc70.png)

30.01.2017: log format were updated to make it easer export to CSV and analysis so it slightly different than shown on the video above.
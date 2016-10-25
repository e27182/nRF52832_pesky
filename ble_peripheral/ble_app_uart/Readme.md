# BLE: HID UART

Adaptation of ble_app_uart example from Nordic Semi SDK. It reuses source code from original folder in SDK, just Makefile is adapted.

After flashing you should be able to connect to your board with "nRF Toolbox" app from App Store (on Android, I beleave there exists similar app for iOS from Nordic Semiconductor),
and use UART mode to talk to the board via BLE. Messages will be forwarded to board's UART. 

Please, see video below:

![image](https://cloud.githubusercontent.com/assets/14309815/19699502/3f0c7682-9afc-11e6-80f5-18a5873445f2.gif)
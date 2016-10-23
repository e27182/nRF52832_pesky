# UART

This example shows basic usage of UART. And it is same as nRF52 SDK example, only Makefile were modified to used appropriate BSP include, board name, flash target.

For exact pins used please look at **projects/include/custom_board.h**.

Also, don't forget to you should switch pins when connecting to COM port: RX should go to TX, TX to RX, CTS -> RTS and RTS -> CTS. 

Compile, flash, connect to COM port (I used putty with next settings):

![image](https://cloud.githubusercontent.com/assets/14309815/19628151/f503b782-995f-11e6-89bf-109c17cf663b.png)

And you should see something like this:

![image](https://cloud.githubusercontent.com/assets/14309815/19628159/2ab25adc-9960-11e6-8581-5bfd97764f87.png)
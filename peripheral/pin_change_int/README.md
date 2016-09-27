# Interrupts

This example shows basic usage of interrupts (adapted from example from official SDK).
It removes protection from pins 9, 10 (used as NFC1,2 by default) so that you can use them as ordinary GPIO.
Interrupt configured on pin 9 to toggle led on pin 22. So if you connect pin 9 to GND led will toggle. 

Compile, flash, play with pin 9 and you should see something like that:

![image](https://cloud.githubusercontent.com/assets/14309815/18880402/679c1bd4-84e0-11e6-8744-2b15b10cffab.gif)
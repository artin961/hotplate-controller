# reflow-iron
DIY hotplate for reflow soldering using modified clothing iron


The controller is using PID to control the heater. PID parameters are user adjustable and being saved to EEPROM after 30 seconds from the last change.

![photo_2023-08-18_00-22-40](https://github.com/artin961/hotplate-controller/assets/11786511/601190d2-cae6-40f8-99aa-0211ecc8921f)

Solid state relay is supposed to be connected on PIN 11
For reading the temperature MAX6675 and a Thermocouple-K are used. 


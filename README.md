# Self Balancing Robot Arduino sketch

**Note: This readme is a work in progress - the project is still under development**

Sketch for a self balancing arduino robot using an [Arduino Uno](http://www.dx.com/pt/p/uno-r3-atmega328p-development-board-for-arduino-402904?Utm_rid=60225380&Utm_source=affiliate), a [MPU6050](http://www.dx.com/p/gy-521-mpu6050-3-axis-acceleration-gyroscope-6dof-module-blue-154602?Utm_rid=60225380&Utm_source=affiliate), [NEMA 17 motors](http://www.dx.com/pt/p/geeetech-1-8-degree-nema-14-35-byghw-stepper-motor-for-3d-printer-black-386069?Utm_rid=60225380&Utm_source=affiliate) and two [A4988](http://www.dx.com/pt/p/3d-printer-a4988-arduino-reprap-stepper-motor-driver-265980?Utm_rid=60225380&Utm_source=affiliate) drivers.

## Wiring

#### MPU6050 wiring 
  * GND/VCC shared with the rest of the circuit - perhaps change GND to second arduino gnd.
  * SDA/SCL to A4/A5 respectivly

#### NEMA 17 wiring 
  [NEMA 17 motors](http://www.dx.com/pt/p/geeetech-1-8-degree-nema-14-35-byghw-stepper-motor-for-3d-printer-black-386069?Utm_rid=60225380&Utm_source=affiliate) with 4 wires scheme:
<p align="center">
  <img src="http://i.imgur.com/DTpdCjG.jpg"/>
</p>
#### A4988 wiring with NEMA 17 bipolar motors (4 wires)
  Both motors are wired to their own A4988
  * VMOT/GND external power supply - GND can be shared with the rest of the circuit;
  * 2B - Black Wire;
  * 2A - Green Wire;
  * 1A - Blue Wire;
  * 1B - Red Wire;
  * GND/VDD shared with the rest of the circuit logic power supply;
  * MS1 to MS3 all to HIGH to allow microstepping;
  * STEP and DIR - the input PINs assigned to the step and dir functions.
<p align="center">
  <img src="http://a.pololu-files.com/picture/0J3360.1200.png"/>
</p>

Current wiring of the A4988 and Arduino - motor wires ***not*** connected for image readability sake.
* Orange and Yellow wires are step and dir (respectively)
* All the MS are connected to Arduino 5v

<p align="center">
  <img src="http://i.imgur.com/FvkU5qE.jpgg" width="530"/>
</p>

## Current state of the robot
* Current state as of: 22/01/2016 - youtube video [here](https://www.youtube.com/watch?v=yMFi8TMg03o)
* Current state as of: 03/02/2016 - youtube video [here](https://www.youtube.com/watch?v=o339cVn5oNA)


<p align="center">
  <img src="http://i.imgur.com/UHWR2By.jpg" width="300"/>
</p>

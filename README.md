Self Balancing Robot Arduino sketch

Sketch for a self balancing arduino robot using an [MPU6050](http://www.dx.com/p/gy-521-mpu6050-3-axis-acceleration-gyroscope-6dof-module-blue-154602?Utm_rid=60225380&Utm_source=affiliate), NEMA 17 motors and two A4988 drivers.

- Problems:
  - MPU6050 stop working as soon as motors are switch one.
    - Rewiring needed. Shorten every wire, specially the ones comming from the motors (they are way too long).
    - Move MPU away from all the wires - to a different breadboard if possible
    - Arduino needs to be turned 

- Wiring of MPU6050
  - GND/VCC shared with the rest of the circuit - perhaps change GND to second arduino gnd.
  - SDA/SCL to A4/A5 respectivly

- Wiring A4988 with NEMA 17 bipolar motors (4 wires)
  - VMOT/GND external power supply - GND can be shared with the rest of the circuit
  - 2B - Black Wire
  - 2A - Green Wire
  - 1A - Blue Wire
  - 1B - Red Wire
  - GND/VDD to shared with the rest of the circuit logic power supply 
  - MS1 to MS3 all to HIGH for more precision
  - STEP and DIR - the input PINs assigned to the step and dir functions 


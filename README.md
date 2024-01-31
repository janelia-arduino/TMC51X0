- [Library Information](#org3e5e3c7)
- [Stepper Motors](#org98a3e76)
- [Stepper Motor Controllers and Drivers](#orgfb036b0)
- [Examples](#org79c28fc)
- [Hardware Documentation](#org8e683ae)
- [Host Computer Setup](#org4da25f6)

    <!-- This file is generated automatically from metadata -->
    <!-- File edits may be overwritten! -->


<a id="org3e5e3c7"></a>

# Library Information

-   **Name:** TMC51X0
-   **Version:** 0.1.0
-   **License:** BSD
-   **URL:** <https://github.com/janelia-arduino/TMC51X0>
-   **Author:** Peter Polidoro
-   **Email:** peter@polidoro.io


## Description

TMC5130 and TMC5160 are high-performance ICs combining both a stepper motor controller and driver.

Both have serial communication interfaces, either SPI or UART, and combine a flexible ramp generator for automatic target positioning with an advanced stepper motor driver.

The TMC5130 and TMC5160 are register compatible allowing the same driver library to work with both.

The TMC5130 includes MOSFETs inside the IC to reduce part count and circuit size for driving motors up to 1.64A RMS per coil.

The TMC5160 uses external MOSFETs to drive higher current motors from 1A to several 10A per coil.

<img src="./images/trinamic_wiring-TMC51X0-description.svg" width="1200px">


<a id="org98a3e76"></a>

# Stepper Motors

From Wikipedia, the free encyclopedia:

A stepper motor, also known as step motor or stepping motor, is a brushless DC electric motor that divides a full rotation into a number of equal steps. The motor's position can be commanded to move and hold at one of these steps without any position sensor for feedback (an open-loop controller), as long as the motor is correctly sized to the application in respect to torque and speed.

[Wikipedia - Stepper Motor](https://en.wikipedia.org/wiki/Stepper_motor)


<a id="orgfb036b0"></a>

# Stepper Motor Controllers and Drivers

Stepper motors need both a controller and a driver. These may be combined into a single component or separated into multiple components that communicate with each other, as is the case with the TMC51X0 stepper motor driver. One controller may be connected to more than one driver for coordinated multi-axis motion control.


## Stepper Motor Controller

A stepper motor controller is responsible for the commanding either the motor kinetics, the torque, or the motor kinematics, the position, speed, and acceleration of one or more stepper motors.


## Stepper Motor Driver

A stepper motor driver is responsible for commanding the electrical current through the motor coils as it changes with time to meet the requirements of the stepper motor controller.


<a id="org79c28fc"></a>

# Examples


## Wiring


### TMC5130-BOB

1.  Teensy 4.0

    <img src="./images/trinamic_wiring-TMC51X0-tmc5130_bob-teensy40.svg" width="1200px">


### Wiring Documentation Source

<https://github.com/janelia-kicad/trinamic_wiring>


<a id="org8e683ae"></a>

# Hardware Documentation


## Datasheets

[Datasheets](./datasheet)


<a id="org4da25f6"></a>

# Host Computer Setup


## GNU/Linux


### Drivers

GNU/Linux computers usually have all of the necessary drivers already installed, but users need the appropriate permissions to open the device and communicate with it.

Udev is the GNU/Linux subsystem that detects when things are plugged into your computer.

1.  Download rules into the correct directory

    1.  Teensy
    
        ```sh
        curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules | sudo tee /etc/udev/rules.d/00-teensy.rules
        ```

2.  Restart udev management tool

    ```sh
    sudo service udev restart
    ```

3.  Ubuntu/Debian users may need to add own “username” to the “dialout” group

    ```sh
    sudo usermod -a -G dialout $USER && sudo usermod -a -G plugdev $USER
    ```

4.  After setting up rules and groups

    You will need to log out and log back in again (or reboot) for the user group changes to take effect.
    
    After this file is installed, physically unplug and reconnect your board.

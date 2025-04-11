- [Library Information](#orgfbfae7e)
- [Stepper Motors](#orgdc2c6a9)
- [Stepper Motor Controllers and Drivers](#org7f33f9b)
- [Motor Current](#org0035cd0)
- [Examples](#org721cfed)
- [Hardware Documentation](#org90d0371)
- [Host Computer Setup](#orgfdc0cba)

    <!-- This file is generated automatically from metadata -->
    <!-- File edits may be overwritten! -->


<a id="orgfbfae7e"></a>

# Library Information

-   **Name:** TMC51X0
-   **Version:** 2.0.0
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

<img src="./images/trinamic-wiring-TMC51X0-description.svg" width="1920px">


<a id="orgdc2c6a9"></a>

# Stepper Motors

From Wikipedia, the free encyclopedia:

A stepper motor, also known as step motor or stepping motor, is a brushless DC electric motor that divides a full rotation into a number of equal steps. The motor's position can be commanded to move and hold at one of these steps without any position sensor for feedback (an open-loop controller), as long as the motor is correctly sized to the application in respect to torque and speed.

[Wikipedia - Stepper Motor](https://en.wikipedia.org/wiki/Stepper_motor)


<a id="org7f33f9b"></a>

# Stepper Motor Controllers and Drivers

Stepper motors need both a controller and a driver. These may be combined into a single component or separated into multiple components that communicate with each other, as is the case with the TMC51X0 stepper motor driver. One controller may be connected to more than one driver for coordinated multi-axis motion control.


## Stepper Motor Controller

A stepper motor controller is responsible for the commanding either the motor kinetics, the torque, or the motor kinematics, the position, speed, and acceleration of one or more stepper motors.


## Stepper Motor Driver

A stepper motor driver is responsible for commanding the electrical current through the motor coils as it changes with time to meet the requirements of the stepper motor controller.


<a id="org0035cd0"></a>

# Motor Current

Several settings affect the magnitude of the motor current, depending on what mode the driver is in at a given velocity.


## global\_current\_scaler

Global scaling of Motor current. This value is multiplied to the current scaling to adapt a drive to a certain motor type. This value should be chosen before tuning other settings because it also influences chopper hysteresis.


### Real Units

Range 0..100 percent > 50 percent recommended for best results


### Chip Units

Range 0..255 0: full scale 1..31 not allowed for operation 32..255 32/256..255/256 of maximum current


## run\_current


### Automatic Current Control Enabled

run\_current scales the current magnitude based on readings from the sense resistors.


### Automatic Current Control Disabled

The current magnitude are not enforced by regulation using the sense resistors, but run\_current scaled the PWM amplitude.


## pwm\_offset and pwm\_gradient


### Automatic Current Control Enabled

pwm\_offset and pwm\_gradient are used as initial values for automatic scaling.


### Automatic Current Control Disabled

pwm\_offset and pwm\_gradient scale the PWM amplitude.


<a id="org721cfed"></a>

# Examples


## Wiring


### Microcontroller

<img src="./images/trinamic-wiring-TMC51X0-microcontroller.svg" width="1920px">


### SPI

1.  TMC51X0-BOB

    1.  Teensy 4.0
    
        <img src="./images/trinamic-wiring-TMC51X0-tmc51x0-spi-tmc51x0-bob-teensy40.svg" width="1920px">
    
    2.  Pico
    
        <img src="./images/trinamic-wiring-TMC51X0-tmc51x0-spi-tmc51x0-bob-pico.svg" width="1920px">

2.  TMC51X0-EVAL

    1.  Pico
    
        <img src="./images/trinamic-wiring-TMC51X0-tmc51x0-spi-tmc51x0-eval-pico.svg" width="1920px">

3.  Wiring Documentation Source

    <https://github.com/janelia-kicad/trinamic-wiring>


<a id="org90d0371"></a>

# Hardware Documentation


## Datasheets

[Datasheets](./datasheet)


<a id="orgfdc0cba"></a>

# Host Computer Setup


## Download this repository

<https://github.com/janelia-arduino/TMC51X0.git>

```sh
git clone https://github.com/janelia-arduino/TMC51X0
```


## PlatformIO


### Install PlatformIO Core

<https://docs.platformio.org/en/latest/core/installation/index.html>

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install pip --upgrade
pip install platformio
pio --version
```


### 99-platformio-udev.rules

Linux users have to install udev rules for PlatformIO supported boards/devices.

1.  Download udev rules file to /etc/udev/rules.d

    ```sh
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    ```

2.  Restart udev management tool

    ```sh
    sudo service udev restart
    ```

3.  Add user to groups

    ```sh
    sudo usermod -a -G dialout $USER && sudo usermod -a -G plugdev $USER
    ```

4.  Remove modemmanager

    ```sh
    sudo apt-get purge --auto-remove modemmanager
    ```

5.  After setting up rules and groups

    You will need to log out and log back in again (or reboot) for the user group changes to take effect.
    
    After this file is installed, physically unplug and reconnect your board.


### Compile the firmware

1.  Gnu/Linux

    ```sh
    make firmware
    ```

2.  Other

    ```sh
    pio run -e teensy40
    ```


### Upload the firmware

1.  Gnu/Linux

    ```sh
    make upload
    ```

2.  Other

    ```sh
    pio run -e teensy40 -t upload
    ```


### Serial Terminal Monitor

1.  Gnu/Linux

    ```sh
    make monitor
    ```

2.  Other

    ```sh
    pio device monitor --echo --eol=LF
    ```


## Arduino Ide


### Download

<https://www.arduino.cc/en/software>


### Additional Boards Manager URLs

File > Preferences

    https://www.pjrc.com/teensy/package_teensy_index.json
    https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json


### Add Board Support Packages

-   Teensy by Paul Stroffregen
-   Raspberry Pi Pico/RP2040 by Earle F Philhower, III

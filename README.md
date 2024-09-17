# PC_Arduino_Fan_Controller
Arduino Fan and RGB Controller

A repository in shambles!
This is heavily under construction and many things are yet to be added/fixed/sorted out.

This project is for an Arduino-Nano based Fan and RGB controller, to fit in an old repurposed DVD Drive cage.

Hardware features:
- It can control 6 fans with PWM, of which channels 1-4 can use variable voltage DC.
- It controls three separate 12v RGB channels. 2 are 12v, and 1 is 5v for LEDs to fit in the case.
- RGB hard drive light - Two 2-pin connectors are used to use 2 HDD light inputs (I have a SAS card with a HDD light output). This controls the second 5v output.
- Support for 4 temperature sensors.
- Infra-red remote support, currently set to use the 44 button remotes that come with cheap Chinese RGB kits.
- Light sensor for auto LCD screen dimming.
- 16x2 I2C LCD display, to display messages.
- Read fan RPM from all channels.
- Serial communication to PC over USB.
- FRAM and EEPROM used for storing settings. EEPROM is used as a backup but must be restored manually with a serial command.
- A buzzer for alerts. I have not programmed this yet.
- Red LED indicator, typically lit up for serial transfers, IR commands and it will flash if the scheduler runs behind.
- Real-time clock so the fan controller can display the time even after being powered off.


Code features:
- Can set custom fan profiles, set using an average of 8 temperature inputs (4 built-in and overrideable, 8 can be sent from the computer).
    - Specifically, fans can all be set from 0 to 100%, regardless of temperature.
- Can set custom RGB profiles, including a 10 colour sequence, or a range of random values.
- Can send messages to the LCD display.
- Can custom strings for the fan controller.
- Can read fan RPM and temperature values to PC.


State of things:
- Most of the Arduino code is feature complete.
- The PCB design needs to be updated, but can be bodged to a working state.
- Component values are often incorrect. A parts list needs to be made.
- Basic Python Serial interaction is mostly working.


Next major tasks:
- Improve documentation and add a creation/installation guide.
- Create a GUI in Python to enable easy setting of RGB and fan profiles, and read data.


If you have taken an interest in this project and something's not right, please raise an issue.
That way I should get an email and I will respond as soon as I can.

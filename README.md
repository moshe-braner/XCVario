# XCVario - Sunton branch

Software for ESP32 based lean Variometer system with open data interface featuring OpenVario, Cambridge, Borgelt and XCVario format.

This branch is for running on the Sunton ESP32-2432S028 board, which:
* does NOT have any of the sensors, nor the rotary switch, CANbus, etc
* has an ESP32 processor plus WiFi/BT (WROOM module)
* has the same TFT display controller (ILI9341) as the XCvario (but pins differ)
* has a touch sensor overlaying the display
* has a (different) audio output module (but without the "poti")
* has an SD card slot

Implemented use of the touch screen in lieu of the missing rotary switch:  Touch near the middle of the screen to mimic the rotary pushbutton.  Touch near the top or the bottom of the screen to navigate through menu items or to change values.

Serial communications ("S1", TTL level) possible on the Sunton board via GPIO pins 35 & 22 which are accessible in the board's expansion connector.  Or use WiFi instead.

The purpose of compiling for this hardware is:
* to test UI features developed for XCvario, without access to an actual XCvario
* possibly to use as a slave XCvario in a 2-seater

Possible future features for use in a 2-seater include:
* Split audio volume of master and slave devices
* Get pitch and bank synced from master XCvario
* Use the SD card slot to record IGC flight logs

In the source code, parts to be skipped are enclosed in #if !defined(NOSENSORS).  Parts that are specific to the Sunton board are enclosed in #ifdef SUNTON28 (since there are other board models from the same and other brands).

To compile this version for use on an actual XCvario, remove the definitions of NOSENSORS and SUNTON28 from the file main/CMakeLists.txt (not the CMakeLists.txt in the root folder of the repo).

New features implemented so far (that can be also used on the actual XCvario):

* Reorganized setup menus
* Simplified data routing settings
* Expanded data routing capabilities
* Optional simpler FLARM warning screen
* Alternative horizon display screen

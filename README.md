# XCVario - Moshe Braner's branch

https://github.com/moshe-braner/XCVario

Software for ESP32 based lean Variometer system with open data interface featuring OpenVario, Cambridge, Borgelt and XCVario format.

This branch aims to offer some features not accepted (so far, anyway) into the "factory" XCvario firmware.  New features implemented so far:

* Reorganized the setup menus to make them more logical and intuitive
* Can escape from deep in the menus to the vario screen via long-press
* Simplified the data routing settings - each route appears only once
* Expanded data routing capabilities - more cross-device routes, and TCP port 2000
* More audio style options including "RICO VACS" inspired ticking sound
* Optional simpler FLARM warning screen, easier to interpret at a glance
* Alternative horizon display screen - uncluttered and easy to view
* A fix to allow use of Bluetooth Low Energy (BLE) connection to XCsoar

The source code is in the "dev" branch (which is the default).

The latest binary is named XCvario-MBdev-xxxxx and can be found in this folder:
https://github.com/moshe-braner/XCVario/tree/dev/images

If you have problems with my version, let me know.  If you like it, let XCvario know :-)

For more information on the reorganized menus, see:
https://github.com/moshe-braner/XCVario/blob/dev/handbook/menu_highlights.txt
and
https://github.com/moshe-braner/XCVario/blob/dev/handbook/menu_plan_not_quite_up_to_date.xlsx

For more information on the enhanced routing, see:
https://github.com/moshe-braner/XCVario/blob/dev/handbook/newroutes.txt
and
https://github.com/moshe-braner/XCVario/blob/dev/handbook/routes.xlsx

For more information on the alternative horizon display, see:
https://github.com/moshe-braner/XCVario/blob/dev/handbook/horizon_notes.txt

About the simplified FLARM warning screen and sound:
* choice of one-icon screen
* separate audio & visual warnings
* choice of short audio while visual continues
* audio warning sounds again if alarm level rises
* hold-off via pressing the knob


This branch also aims to run on the Sunton ESP32-2432S028 board, which:
* does NOT have any of the sensors, nor the rotary switch, CANbus, etc
* has an ESP32 processor plus WiFi/BT (WROOM module)
* has the same TFT display controller (ILI9341) as the XCvario (but pins differ)
* has a touch sensor overlaying the display
* has a (different) audio output module (but without the "poti")
* has an SD card slot
Implemented use of the touch screen in lieu of the missing rotary switch:  Touch near the middle of the screen to mimic the rotary pushbutton.  Touch near the top or the bottom of the screen to navigate through menu items or to change values.

The purpose of compiling for this hardware is:
* to test UI features developed for XCvario, without access to an actual XCvario
* possibly to use as a slave XCvario in a 2-seater

Possible future features for use in a 2-seater include:
* Get pitch and bank synced from master XCvario
* Use the SD card slot to record IGC flight logs

In the source code, parts to be skipped are enclosed in #if !defined(NOSENSORS).  Parts that are specific to the Sunton board are enclosed in #ifdef SUNTON28 (since there are other board models from the same and other brands).

To compile this version for the Sunton board, add these definitions of NOSENSORS and SUNTON28 to the end of the file main/CMakeLists.txt (not the CMakeLists.txt in the root folder of the repo):
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DNOSENSORS")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC "-DSUNTON28")

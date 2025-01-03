# miyoostick

A userland joystick application that reads data from `/dev/ttyS1`, auto-calibrates, and publishes a virtual gamepad via `/dev/uinput`.

## Overview

miyoostick.c
- Sets up a virtual gamepad device via /dev/uinput.
- Opens /dev/ttyS1 to read 6-byte frames for the left and right sticks.
- Performs auto-calibration by reading a certain number of frames upon startup to find each stick’s zero point.
- Applies a radial deadzone (configurable via g_deadzone_ratio).
- Publishes the results as analog axes with optional example buttons.

## Quick Start
- Ensure you have a recent Linux system (or Android environment) with uinput support.

- Compile using a command like:
```gcc -o miyoostick miyoostick.c -lpthread```

- Run the application:
 ```./miyoostick```

Verify the new virtual device:
- On Android, you can check via getevent -p.
- On desktop Linux, you can check via evtest or by running:
  ```cat /proc/bus/input/devices```.
  You should see a device named "Miyoo Virtual Stick".

## Notes
- The code will auto-calibrate by reading 50 frames (default) from the serial device, assuming the user is not moving the sticks. You can adjust the CALIBRATION_FRAMES constant to change the number of frames or the g_deadzone_ratio to tweak the deadzone.
- If you wish to enable more buttons or map more axes, you can update the relevant ioctl() calls in setup_uinput_device().

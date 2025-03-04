# Project description

I did not like the blinky stock rear light behavior on the Begode Falcon:
- turn signals based on tilt angle is stupid
- brake light has a bit too agressive blinking

This project contains firmware for an external LED controller for the
rear light. Instead of turn signals, you get stable red.

Unlike turn signals, the brake light wasn't completely useless so that
is reimplemented not by blinking but increasing the rear panel
brightness. This feature requires braking detection. Input for that
was taken from the stock LED controller output, making this
practically a LED strip effect device.

The project is targeting ESP32-C3, namely [Seeed studio XIAO
ESP32C3][1]. Both the LED strip input and output use the RMT (Remote
Control Transceiver) peripheral. Output was straightforward with the
_led\_strip_ API. Input is custom implementation on top of RMT API.

[1]: https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/

# Hardware configuration

| Function   | GPIO  |
|------------|-------|
| LED input  | GPIO2 |
| LED output | GPIO9 |

When input is not detected, the device outputs some predefined test signal.

# Build & flash

Something like:

    source /path/to/esp-idf/export.sh
	idf.py build
	idf.py -p /dev/ttyACM0 flash

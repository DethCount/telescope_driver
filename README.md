# telescope_driver
Dagoma Magis PCB (Atmega2560 - MKS Base 1.0) \
Drive telescope motors \
Allowing command execution through SPI and Serial bus

# telescope_controller
Arduino Uno (VMA101) with Bluetooth Low Energy HM-10 module (VMA338) Connected with telescope_driver through SPI (pins 50, 51, 52, 53) \
Proxies Bluetooth data to SPI

# Commands
Change X motor speed for full speed clockwise
> MOVE X 1.0

Change X motor speed for full speed counter clockwise
> MOVE X -1.0

Stop motors
> STOP


# References
https://github.com/dagoma3d/Marlin-By-Dagoma/ \
https://github.com/arduino/Arduino \
https://github.com/arduino/ArduinoCore-avr/blob/9f8d27f09f3bbd1da1374b5549a82bda55d45d44/variants/mega/pins_arduino.h \
https://www.arduino.cc/en/reference/SPI

# Datasheets
https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf \
https://www.velleman.eu/downloads/29/vma100_vma101_vma102_vma103_a4v01.pdf \
https://www.velleman.eu/downloads/29/vma338_a4v02.pdf

# Power
12V - 5A in MKS Base 1.0 \
connect VCC + GND from SPI into VIN + GND on Arduino Uno

# Testing with Raspberry Pi 3B+
Use [usb-gamepad](https://github.com/DethCount/usb-gamepad) or in terminal:

sudo bluetoothctl
> connect D8:A9:8B:7E:1E:D2 \
> menu gatt \
> select-attribute 0000ffe1-0000-1000-8000-008805f9b34fb \
> acquire-write

AcquireWrite success: fd 7 MTU 23

> quit

Run command through created socket (max command length: 20 ascii characters)
> echo 'MOVE X 1.0' > /proc/$(pgrep bluetoothctl)/fd/7

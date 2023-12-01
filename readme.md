# Bulletin

A little screen that sits at my desk and display whatever I pass to it via serial port

This only uses espressif's esp32 SDK `esp-idf`.  Does not require arduino IDE!

## Build
* `cd firmware`
* `./configure`
* `source ./esp-idf/export.sh`
* `make build`
* `make flash`

## Example
The example provides a simple python script that reads lines from `lyrics.txt` and sends the data serially to the esp32 chip.

[Demo](https://youtu.be/Sfy_Jo7XL9Q?si=Zy2axcjMjMIOdptd)

## references
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/user-guide-devkitm-1.html


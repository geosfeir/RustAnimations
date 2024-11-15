This project is an embedded Rust program for the Adafruit Feather RP2040 and NeoMatrix LED Array.
It is a NO STD environment. Do not try to use any crates that use std, it will not work.

main.rs is the file that contains most of the crate imports, initialization functions, and the main loop.
animations.rs contains all animation structs (Bouncy, Bright, Snake, Spiral) and their respective functions.
usb_manager.rs is taken from the adafruit-feather-rp2040 example repo, it handles connecting to the firmware.

The crates used are all listed in the Dependencies folder.
This program reads accelerometer tilt values from the LIS3DH sensor that is on the board.
Using these tilt values, the NeoMatrix LED display will play 4 different animations.

This program can be compiled by using "cargo build" in the command line.
And it can be run and programmed on the board by using "cargo run" in the same terminal.
You may get many warnings like:
warning: the `-Cinline-threshold` flag is deprecated and does nothing (consider using `-Cllvm-args=--inline-threshold=...`),
But they can all be ignored.

Enjoy!
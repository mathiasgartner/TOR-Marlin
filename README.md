# TOR-Marlin

This repository extends the functionality of the [Marlin 3D Printer Firmware](https://marlinfw.org/) for its usage in the project  [**The Transparency of Randomness (TOR)**](https://github.com/MathiasGartner/TOR).

## Additional functionality

In the TOR project, a magnet, supsended on four cords, needs to be moved precisely in 3D space. This is achieved by using four stepper motors than control the length of the cords.

The Marlin firmware is extended, such that the stepper motor channel usually used for the extruder in 3D printing, can also be used for movement. Additional coordinate transforms needed for positioning the magnet are implemented.
# Code-Samples

The files in this repository are not necessarily related to each other and are also not necessarily compilable. They are simply snap-shots of code that were used in one project or another at some point.

All code was written prior to or in lieu of official/supported releases of example code from part manufacturers for devices that were in some cases still pre-production/experimental.

The target for:
  - motor.c
  - motor.h
  - main.c
  - magnet.c
  - magnet.h
was a TI Stellaris LM4F120H5QR (now the TI Tiva C TM4C123GH6PM) M4 ARM microcontroller.

motor.c & motor.h interfaced two ST L6470 dSPIN stepper motor drivers to the target.
magnet.c & magnet.h interfaced a FET that controlled an electromagnet to the target.
main.c interfaced a TI CC1121 RF tranceiver to the target.

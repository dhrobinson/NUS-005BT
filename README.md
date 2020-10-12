# NUS-005BT: N64 Bluetooth controller mod

## Overview

The N64 controller is a classic piece of gaming hardware. Some games just don't feel right with modern controllers, so why not bluetooth-ify an original N64 controller?

A lot of work was done to read the N64 controller with an Arduino, which can then output over USB. This project will add on a Bluetooth module, as well as a couple of other tweaks.

The finished build fits inside an original Rumble Pak* and runs from 2xAA batteries to mimic the original feel and weight of the controller.

**I hope to add rumble support in the future*

## What you'll need

- NUS-005 N64 controller
- NUS-013 N64 Rumble Pak
- 3.3v / 16mhz Arduino (link)
- RN42 I/RN Bluetooth module (link)
- Soldering iron, solder, wire
- Patience & steady hands..!

## Wiring overview

The N64 controller operates using three wires: GND, Data and 3.3v. We can perform a small internal mod for two reasons:

1. Reroute the data line into the Rumble Pak
2. Push a power line back into the Rumble Pak to power the Arduino

The Rumble Pak will have one specific mod:

1. Push the power from the batteries into the controller

Wiring it up this way means that the controller will be powered by the batteries, and the Arduino will be powered from a line that runs from the batteries, into the controller, and back out into the Rumble Pak.

This means that when we want to turn the Arduino on or off, we can just insert or remove the Rumble Pak. No switches necessary!

### Schematic

N64 controller
```
PWR --------------- Expansion pin 20
DATA -------------- Expansion pin 25
```

Rumble pak
```
PWR --------------- Expansion pin 31
```

Rumble pak / Arduino
```
RUMBLE PAK             ARDUINO

P20 ---------------------- RAW
GND ---------------------- GND
P25 -------(data)--------- P02

```

Arduino / RN42
```
ARDUINO             RN42

VCC --------------- PWR
GND --------------- GND
P14 --------------- TX
P16 --------------- RX
```

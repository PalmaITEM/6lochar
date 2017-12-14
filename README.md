# 6lochar

Kernel module to create a fake _6LoWPAN_ interface that connects to a character device for sending a receiving frames.

Tested with _4.x.y_ Linux Kernel releases.

## Why?

Even though _6LoWPAN_ is mostly used for _IEEE 802.15.4_ devices, other devices/radios can also benefit from the defined compression mechanisms.
The goal of this project was to demonstrate _6LoWPAN_ over a resource-constrained _VHF_ link.

## How?

This kernel module registers a character device that can be read or written into, which in turn reads and writes _802.15.4_ frames from and to the created _wpan_ interface.

More details on why and how available on https://sinet.dpalma.eu or https://sinet.item.ntnu.no

# Getting started

## Load Module

Simply compile the kernel module with `make` and run `./load_module`.

This should create a new _lowpan_ interface with a random _short address_ in the _pan_id_ **0xbeef**.

The _load_module_ script loads the compiled kernel module and searches the last 5 lines for the path of the registered character device that will used for communication with the _802.15.4 stack_. After creating the character device a new _lowpan_ interface is added to wpan0.

### Caveats

1. The script "assumes" that no other _wpan0_ device already exists and should be adapted otherwise.

2. Sometimes the Kernel never creates an initial address to the _lowpan_ interface and the cause is unknown. In this event an error will be returned by _iproute_ when running `./load_module`. In this case, the only solution is to unload and load the module again.

3. Caveat number 2 is only relevant when full IPHC is desired, otherwise it works fine despite the error.



## Connect 6lochar to a radio/application

To connect the newly created interface to a radio or even an application reading and writing _IEEE 802.15.4_ frames the connector example can be used as **inspiration**.

Specifically, this example provides the option to connect to the OWL radio, using the [NGHAM-SPP protocol](https://github.com/skagmo/ngham) in the serial port _/dev/ttyUSB0_, if completed with a parser for NGHAM-SPP frames.

# To-do list

Apart from a better organisation of some structures, the character device creation follows an outdated method, and should be changed.

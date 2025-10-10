<!--
© 2018, PolySync Technologies, Inc., Shea Newton <snewton@polysync.io>

This file is part of oscc-check

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# OSCC Check

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


  - [Overview](#overview)
  - [Getting Started](#getting-started)
    - [Compatibility](#compatibility)
    - [Dependencies](#dependencies)
    - [Building](#building)
  - [Usage](#usage)
    - [Options](#options)
- [License](#license)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Overview

The purpose of this Python 3 program is to provide basic validation that the vehicle and OSCC are in
a good state and are able to communicate as expected. Ideally it's a method to quickly verify
whether your vehicle is in a good state. By sending control commands on the CAN gateway then polling
for an expected result there, we should be able to tell whether OSCC is functioning properly or
diagnose any issues more readily.

While this program was designed to be run as a stand alone executable it may be imported as a module
should a use-case arise.

Prerequisites for success:

- A vehicle with OSCC installed
- That vehicle powered on (motor on)
- OSCC powered
- A socketcan connection to OSCC's CAN bus from your computer

But what does it do?

1. Enable each OSCC module (brake, steering, throttle).
1. Send commands to increase and decrease brake pressure.
1. Verify that brake pressure reported by vehicle increased or decreased accordingly.
1. Send commands to apply positive or negative torque to steering wheel.
1. Verify that the steering wheel angle increased or decreased accordingly.
1. Disable each OSCC module.

## Getting Started

### Compatibility

Works with OSCC `v1.2.1` and up.

### Linux Dependencies

- `python3` (`sudo apt install python3`)

### Windows Dependencies

- `python3` ([https://www.python.org/downloads/windows/](https://www.python.org/downloads/windows/))
- CAN driver ([Kvaser](https://www.kvaser.com/developer/canlib-sdk/), [PCAN](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1), etc.)

### Building

Install this package's dependencies with the following command:

```bash
python3 setup.py install --user
```

## Usage

`oscc-check.py (-V <vehicle>) [-hdelv] [-b <bustype>] [-c <channel>]`

### Options

```bash
Options:
    -h --help                            Display this information
    -V <vehicle>, --vehicle <vehicle>    Specify your vehicle. Required.
                                         (kia_soul_ev / kia_soul_petrol / kia_niro)
    -d --disable                         Disable modules only, no further checks (overrides enable)
    -e --enable                          Enable modules only, no further checks
    -l --loop                            Repeat all checks, run continuously
    -b --bustype <bustype>               CAN bus type [default: socketcan_native]
                                         (for more see https://python-can.readthedocs.io/en/2.1.0/interfaces.html)
    -c <channel>, --channel <channel>    Specify CAN channel, [default: can0]
    -v --version                         Display version information
```

### Usage Notes

#### Linux

On a Linux system `socketcan` is a default so the `oscc-check.py` can rely on the default settings
for `bustype` and `channel`. After initializing the socketcan interface with:

```bash
 sudo ip link set can0 type can bitrate 500000
 sudo ip link set up can0
```

you can run:

```bash
# Default Linux usage for Kia Soul EV
python3 oscc-check.py -V kia_soul_ev
```

#### Windows

On a Windows system `socketcan` is not available so the `bustype` and `channel` must be specified.

If you've installed the Kvaser SDK you need to run:

```bash
# Default Kvaser CANlib usage for Kia Soul Petrol
python oscc-check.py -c 0 -b kvaser -V kia_soul_petrol
```

Using PCAN drivers you can run:

```bash
# Default PEAK PCAN-USB usage for Kia Niro
python oscc-check.py -c PCAN_USBBUS1 -b pcan -V kia_niro
```

### Running Tests

`python3 setup.py test`

# License

© 2018, PolySync Technologies, Inc., Shea Newton <snewton@polysync.io>

[MIT](https://github.com/PolySync/oscc-check/blob/master/LICENSE)

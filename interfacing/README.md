## A complete set of tools for interfacing with the robot.

### OSCC Test Suite: Testing suite for the Eve hardware.
- oscc_check: cloned from https://github.com/PolySync/oscc-check but it's no longer maintained so it is not added as a subrepo. Tests the full oscc firmware system, through physical actuator trials and measuring the resulting changes in wheel speed, brake pressure and steering angle.

- car_monitor: directly listens on the can bus to give you a live terminal gui for all measured values and errors. (does not fill up the terminal screen!)

- oscc_candump_decoder: A python script to decode static can files or live can network packets
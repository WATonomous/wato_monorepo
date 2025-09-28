# Camera Bringup

## Computer specific 

The cameras are plugged into a switch that is connected to the main computer.

Type `ipconfig` to see network interfaces.

The switch is either on `enp7s0f0` or `enp7s0f1`, where:

en = Ethernet
p7 = PCI bus 7
s0 = slot 0
f1 = function 1

Key things to check for:
- Default and max recieve buffer size of 10485760
- Maximum Transmission Unit (MTU) value is 9000 for the interface:


## Camera Specific

### Blackfly GigE (RGB, mounted NW, N, NE of sensor rack)



#### Lens (Fujinon CF12.5HA-1)

To write: tuning and fixing focus
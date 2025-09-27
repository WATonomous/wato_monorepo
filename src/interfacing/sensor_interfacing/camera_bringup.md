# Camera Bringup

## Computer specific 

The cameras are plugged into a switch that is connected to the main computer.

Type `ipconfig` to see network interfaces.

The switch is either on `enp7s0f0` or `enp7s0f1`, where:

en = Ethernet
p7 = PCI bus 7
s0 = slot 0
f1 = function 1

Ensure the Maximum Transmission Unit (MTU) value is 9000 for the interface:

```sh
4: enp7s0f1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9000 qdisc mq state UP mode DEFAULT group default qlen 1000
    link/ether 1a:4b:23:17:63:dc brd ff:ff:ff:ff:ff:ff
```

## Camera Specific

### Blackfly GigE (RGB, mounted NW, N, NE of sensor rack)



#### Lens (Fujinon CF12.5HA-1)

To write: tuning and fixing focus
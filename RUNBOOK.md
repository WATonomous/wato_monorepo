# Testing Eve Vehicle in Real Life

**NOTE** This is meant to be a living document of how WATonomous runs the EVE platform. Steps are subject to change and MUST BE UPDATED WHEN NECESSARY.

## DANGER
- DO NOT TOUCH THE LEADS OF THE MAIN CAR BATTERY, you will die
  - Located under a metal plate in the middle of the backseat
- DO NOT TOUCH THE LEADS OF THE SMALL CAR BATTERY AND THE CHASSIS WITH LEFT AND RIGHT HAND, you will die

## General Procedure
### Charging
To charge the car, open the front and use the emergency latch to open the car charging dock.

When charging the car, the following must be true:
1. estop is pushed in
1. compute rack fuse must be disconnected (as to not slowly discharge small car battery), done by pushing the yellow button on the fuse itself
1. car peripherals are not on (**WARNING: the trunk being open will discharge the battery, so if trunk is to be left open, FULLY DISCONNECT THE SMALL BATTERY**)
1. computer peripherals are either moved/connected to an external monitor, mouse, keyboard, or left in the car

### Parking Car
If you want to park the car for a prolonged period of time and not connect it to a charger. You must make sure the following is true:
1. estop is pushed in
1. compute rack fuse must be disconnected (as to not slowly discharge small car battery), done by pushing the yellow button on the fuse itself
1. car peripherals are not on (**WARNING: the trunk being open will discharge the battery, so if trunk is to be left open, FULLY DISCONNECT THE SMALL BATTERY**)
1. computer peripherals are either moved/connected to an external monitor, mouse, keyboard, or left in the car
1. (Optional) Compute rack sensors and computer are connected to a wall plug. Ethernet to the computer is connected to school's ethernet.

> Generally, the following tend to discharge the car battery:
>- Open trunk
>- Compute rack connected and on (Sinewave generator will also discharge battery)
>- OSCC boards (discharging can be stopped with estop)
>
> All we trying to do with these steps is to stop the small car battery from discharging

### Preparing Car for Test Track
When preparing the car for the test track, do the following:
1. keep estop pushed in
1. reconnect small car battery
1. reconnect compute rack fuse by pushing the yellow lever in
1. connect sensor power bar to sine wave generator (blue box)
1. connect computer to sine wave generator
1. turn on the car and proceed to go to test track
1. **Once at the test track** disconnect the car battery, wait 10 sec, and reconnect the car battery (this is a hack because for some reason our boards start to fault if we've driven the car ourselves for too long, you can use this tactic whenever our teleop setup doesn't work. We need to look into this)
1. turn car back on, and drive roughly 200m in any direction (so that we can hone in the gps, else localization and pid controls wont work)
1. proceed to up the entire software start with `watod` (enabling all profiles and upping)

## Debugging

### The car won't turn on
This is most likely because the small car battery has died, and someone failed to follow the steps above and should be punished.

**To check** measure the voltage of the small car battery. If it is significantly less than 13V, (like 3-2.6V) then we have drained the small car battery and we need to replace it.

**To bring back car batteries from the dead, there is a special snap on jumper in AVRIL that can recharge the car battery** You can politely ask for it. It's really powerful. Car batteries brought down to 3V need a really powerful jumper.

**If the battery is unrecoverable** get it replaced by warranty which will take awhile, or, buying a new one for the kia soul at costco or canadian tire.

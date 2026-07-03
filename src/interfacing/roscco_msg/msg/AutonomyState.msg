# Autonomy engagement state of the OSCC steering actuator.
#
# DISABLED:    OSCC boards are disabled, the driver has full control.
# ENGAGING:    a graceful handover to autonomy is in progress; command authority
#              (steering torque and throttle) is being ramped up from zero.
#              Autonomy commands are applied but scaled down during this phase.
# ENGAGED:     normal operation, autonomy commands are applied at full authority.
# DISENGAGING: a graceful handover to the driver is in progress; steering torque
#              is being ramped to zero. Autonomy commands are ignored during this phase.
uint8 ENGAGED=0
uint8 DISENGAGING=1
uint8 DISABLED=2
uint8 ENGAGING=3

std_msgs/Header header
uint8 state

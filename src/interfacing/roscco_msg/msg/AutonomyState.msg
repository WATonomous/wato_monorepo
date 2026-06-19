# Autonomy engagement state of the OSCC steering actuator.
#
# ENGAGED:     normal operation, autonomy commands are applied.
# DISENGAGING: a graceful handover is in progress; steering torque is being
#              ramped to zero. Autonomy commands are ignored during this phase.
# DISABLED:    OSCC boards are disabled, the driver has full control.
uint8 ENGAGED=0
uint8 DISENGAGING=1
uint8 DISABLED=2

std_msgs/Header header
uint8 state

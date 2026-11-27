# One hardware trigger pulse, as reported by the camera trigger board (STM32).
# Published by camera_trigger_node on /camera_trigger, one message per pulse.
# Pulses that were never reported (lost packets) are never published.

# stamp: the time the pulse fired, on the system clock (UTC).
#   Computed by snapping the receive time onto the PPS grid (second + pulse_idx / pulses_per_second).
#   It is the exact pulse time only when time_valid is true.
# frame_id: "camera_trigger"
std_msgs/Header header

# Pulse counter from the board. +1 on every pulse, 0 after a board reboot, wraps at 2^32.
uint32 seq

# Seconds counted by the board since boot (+1 on each PPS edge, or every pulses_per_second pulses without PPS).
uint32 pps_count

# Pulse index within the current second. 0 is the pulse fired on the PPS edge.
uint8 pulse_idx

# True if the board's pulse train is locked to the GPS PPS signal.
bool pps_locked

# True only if the stamp was snapped to the PPS grid and passed every check
# (PPS locked, receive delay within limits, second counter consistent).
# Assumes the system clock is synced to GPS (chrony, issue #560).
# A clock error of a whole number of seconds cannot be detected here.
bool time_valid

# Bit k set = strobe input STB(k+1) saw a camera start exposing for this pulse. 0x0FFF = all 12.
uint16 strobe_mask

# False if the board does not measure strobes for this pulse (strobe_mask is then 0).
bool strobe_mask_valid

# System time at which the report was received (kernel receive timestamp when available).
builtin_interfaces/Time receive_time

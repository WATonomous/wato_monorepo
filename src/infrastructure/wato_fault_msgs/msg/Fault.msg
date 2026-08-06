std_msgs/Header header

# Severity: LOWER number == MORE severe.
uint8 CRITICAL=1   # Unsafe to continue driving. Disengage autonomy now.
uint8 SEVERE=2     # Major degradation; autonomy may need to hand back control.
uint8 MODERATE=3   # Degraded but driveable.
uint8 MILD=4       # Informational / nuisance.

uint8 level

# Where the fault came from -- node name, subsystem, or hardware id.
string source

# Description of what went wrong.
string message

# Optional stable identifier for this fault within `source`. Reports sharing the
# same (source, id) replace one another instead of accumulating. Empty means the
# `message` text itself is used as the identifier.
string id

# True = fault is active (raise). False = the source is explicitly clearing it.
# Faults are also cleared automatically if not re-published within the
# collector's fault_timeout_s.
bool active true

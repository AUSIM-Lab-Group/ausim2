# Discrete simulation event command routed into the simulator runtime.

builtin_interfaces/Time stamp
string vehicle_id
string ros_namespace

uint8 KIND_NONE=0
uint8 KIND_RESET_SIMULATION=1
uint8 KIND_GENERIC_EVENT=2

uint8 kind
string event_name
uint64 sequence

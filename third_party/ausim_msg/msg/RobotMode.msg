# Structured robot mode snapshot for external consumers.

builtin_interfaces/Time stamp
string vehicle_id
string ros_namespace

uint8 TOP_STATE_SAFE=0
uint8 TOP_STATE_MANUAL_READY=1
uint8 TOP_STATE_MANUAL_ACTIVE=2
uint8 TOP_STATE_AUTO=3
uint8 TOP_STATE_FAULT=4

uint8 ACK_NONE=0
uint8 ACK_SUCCESS=1
uint8 ACK_UNSUPPORTED=2
uint8 ACK_FAILED=3

uint8 top_state
string sub_state
bool accepts_motion
uint64 last_discrete_command_sequence
uint8 last_discrete_command_status

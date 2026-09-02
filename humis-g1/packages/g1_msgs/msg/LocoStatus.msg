uint8 MODE_UNKNOWN=0
uint8 MODE_DAMP=1
uint8 MODE_STAND=2
uint8 MODE_WALK=3
uint8 MODE_ZERO_TORQUE=4
std_msgs/Header header
uint8 mode
float32 battery_soc
# halted: soft-stop / halt latch engaged - true after /g1/halt, a non-balancing
# set_mode, or an auto-halt on loss of FSM 500; false while driving in walk mode.
# Only FSM 500 (MODE_WALK) has balance control; MODE_DAMP/STAND/ZERO_TORQUE mean
# no balance control (the robot is down / will fall).
bool halted
int32 error_code

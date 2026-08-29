uint8 RELEASED=0
uint8 FORWARD=1
uint8 FORWARD_LEFT=2
uint8 FORWARD_RIGHT=3
uint8 LEFT_TURN=4
uint8 RIGHT_TURN=5
uint8 REVERSE=6
uint8 REVERSE_LEFT=7
uint8 REVERSE_RIGHT=8

std_msgs/Header header
string session_id
uint64 sequence
# Compatibility projection for existing ROS-only simulation publishers and
# consumers. Protocol-v3 physical intent uses the signed axes below.
float32 steering
float32 forward
float32 lateral
float32 longitudinal
# Maximum absolute steering ratio the operator explicitly delegates to local
# path assistance. Zero preserves direct shared control.
float32 max_steering_assist
uint8 intent_class
bool deadman

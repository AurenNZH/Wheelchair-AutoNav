uint8 STOP=0
uint8 SLOW=1
uint8 CLEAR=2

std_msgs/Header header
string session_id
uint64 intent_sequence
uint8 decision
float32 permitted_forward
float32 permitted_steering
string reason
float32 map_age_ms

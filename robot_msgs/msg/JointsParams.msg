# This message holds all joints params
#
# Arrays should have same size or be empty

Header header
string[] names
bool[] enabled
float32[] lower_limit
float32[] upper_limit
float32[] offset
bool[] reverse
TypeJointMode[] mode
TypePid[] pids
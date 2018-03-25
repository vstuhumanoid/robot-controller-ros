# Command to joints
# - name - names of joints, required
# - positions - position (angle) of joint, required
# - pids - PID-controller settings for joint, optional
#
# Arrays should have same size. pids may be empty (zero size)

string[] names
float32[] positions
Pid[] pids
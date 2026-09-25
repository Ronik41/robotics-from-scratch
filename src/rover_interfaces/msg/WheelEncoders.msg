# Sample acquisition time, not delayed publication time; frame_id = base_link.
std_msgs/Header header
# Monotonic per driver session. Restart resets index and count origin to zero.
uint64 sample_index
# Time since previous acquisition; zero for the first sample.
builtin_interfaces/Duration sample_period
string left_joint_name
string right_joint_name
# Signed cumulative counts since first sample; forward rotation is positive.
# Counts per wheel revolution AFTER quadrature decoding and gearbox scaling.
uint32 counts_per_revolution
int64 left_count
int64 right_count

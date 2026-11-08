# One feature's limit on the vehicle's speed, merged by trajectory_coordinator
# (the strictest fresh constraint wins). Features only ever lower the speed.
std_msgs/Header header
string source                       # feature name, e.g. "acc", "cruise"
uint8 priority                      # informational class (3 = restrictive speed constraint)
bool stop_required                  # reserved for a stop-line / traffic-light feature; not merged yet
float64 stop_distance_m             # reserved, as above
float64 maximum_speed_mps           # the cap: the coordinator clamps the whole speed profile to it
float64 maximum_deceleration_mps2   # reserved, as above
builtin_interfaces/Time valid_until # stale after this time (node clock, sim time in the stack)
float64 confidence                  # 0..1, informational
string reason                       # human-readable, ends up on /planning/binding_constraint

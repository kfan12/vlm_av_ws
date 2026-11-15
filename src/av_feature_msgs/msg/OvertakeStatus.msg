# overtake_manager's state, one message per lidar cloud.
# Along-lane distances are measured along the right lane (the lane being
# overtaken in); + is ahead of the ego's base_link.
std_msgs/Header header

uint8 IDLE=0      # no slow vehicle ahead; the lane path passes through unchanged
uint8 FOLLOW=1    # a slower vehicle ahead is being followed (ACC owns the speed)
uint8 PULL_OUT=2  # shifting the path into the left lane
uint8 PASS=3      # in the left lane, passing
uint8 PULL_IN=4   # shifting back into the right lane ahead of the vehicle
uint8 ABORT=5     # falling back behind the vehicle, then shifting back
uint8 INHIBIT=6   # after an abort: no new overtake for a while
uint8 state
string state_name

float64 offset_m             # commanded lateral offset at the ego [m], + = left
float64 target_offset_m      # where the current shift ends [m]
bool truck_tracked
bool truck_visible           # measured on this cloud (false = predicted, e.g. beside the ego)
float64 truck_rear_rel_m     # truck rear relative to the ego's base_link, along the lane [m]
float64 truck_gap_m          # ego front bumper to truck rear [m] (negative once alongside)
float64 truck_clear_m        # ego rear bumper ahead of the truck's conservative front [m]
float64 truck_speed_mps
float64 truck_length_m
float64 truck_blind_sec      # time since the truck was last measured [s]
float64 left_lane_free_m     # free length of the left lane ahead of the ego's front [m]
float64 cap_mps              # the speed cap published on /feature/overtake/constraint
string reason
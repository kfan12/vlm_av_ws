"""Hand-kept part of the architecture page: where each node sits, how each line is routed,
short labels, and the nodes the code scan cannot see (Gazebo, the EKF, the target-speed
script, planned nodes). Topics, publishers, subscribers and which edges exist come from the
code (tools/avkit/kb_index.py scan) and the setups from local/kb/index.yaml; build.py
reports anything here that no longer matches."""

H = 60
C1, C2, C3, C4 = 215, 420, 680, 940

# setups drawn; acc_ot is the acc setup with the planned nodes switched on
SETUPS = ['vlm', 'aeb', 'lidar', 'acc', 'acc_ot']

NOTES = {
    'vlm': '<b>vlm:</b> the VLM sign chain sets the target speed, the MPC drives /cmd_vel directly. No AEB, no arbiter. The VLM nodes run in their own venv and are started by hand.',
    'aeb': '<b>aeb:</b> a script publishes a fixed target speed. aeb_node watches the depth image; control_arbiter is the only /cmd_vel publisher and lets AEB override the MPC.',
    'lidar': '<b>lidar:</b> the same as aeb, but aeb_node reads the front lidar (120 deg) instead of the depth image.',
    'acc': "<b>acc:</b> the lidar setup plus ACC. trajectory_coordinator clamps the planner's profile to the strictest speed cap (cruise, ACC) and the MPC follows that authorized plan. Features cap speed only; they never steer.",
    'acc_ot': "<b>Overtake (planned):</b> overtake_manager takes over local_planner's input path through a launch remap and adds a required speed cap.",
}

# node id = the code's node name (kb_index scan) or an external id below
NODES = {
    'gazebo': dict(x=20, y=30, w=150, h=660, sim=True, t='Gazebo Fortress', s='sedan, urban course, ros_gz_bridge'),
    'target': dict(x=C1, y=140, w=160, t='speed setpoint', s='run script, fixed target'),
    'lead_vehicle': dict(x=C1, y=290, w=160, s='drives the lead truck'),
    'odom_noise': dict(x=C1, y=555, w=160, h=50, s='wheel odometry noise'),
    'ekf': dict(x=C1, y=630, w=160, h=50, t='ekf_node', s='robot_localization'),
    'vlm_sign_node': dict(x=C2, y=30, w=170, s='Qwen2.5-VL-3B sign reader'),
    'sign_maneuver_node': dict(x=C3, y=30, w=170, s='board latch, maneuver FSM'),
    'overtake_manager': dict(x=C3, y=115, w=170, s='path owner, truck tracker', planned=True),
    'lane_node': dict(x=C2, y=200, w=170, s='HSV paint, ground plane'),
    'local_planner': dict(x=C3, y=200, w=170, s='curvature speed profile'),
    'mpc_tracker_v2': dict(x=C4, y=200, w=165, s='OSQP MPC tracker'),
    'constraint_from_maneuver_node': dict(x=C2, y=300, w=170, t='cruise_constraint', s='target speed to constraint'),
    'acc_node': dict(x=C2, y=390, w=170, s='lidar lead tracking'),
    'trajectory_coordinator_node': dict(x=C3, y=345, w=170, t='trajectory_coordinator', s='strictest valid cap wins'),
    'aeb_node': dict(x=C2, y=490, w=170, s='corridor, TTC, 6 states'),
    'control_arbiter_node': dict(x=C4, y=490, w=165, t='control_arbiter', s='e-stop > AEB > fault > MPC'),
}

# nodes in the code that the drawing leaves out on purpose
IGNORE = {'odom_compare', '_course_demo_watch', 'obstacle_probe', 'aeb_test_driver', 'record_aeb_scenario',
          'record_av_lap', 'sensor_rates', 'pitch_check'}

# nodes the scan cannot see; topics as 'name type', with the setups they exist in
EXTERNAL = {
    'gazebo': dict(pkg='Gazebo + ros_gz_bridge (bridge_sedan.yaml, bridge_lidar.yaml)',
                   d='The simulator and its bridges. Sensor topics come out, /cmd_vel goes in. The lidar has its own bridge, started only when the setup renders the lidar. The lead truck is bridged by the ACC scenario script.',
                   bridges=[('src/robot_bringup/config/bridge_sedan.yaml', ['vlm', 'aeb', 'lidar', 'acc', 'acc_ot']),
                            ('src/robot_bringup/config/bridge_lidar.yaml', ['lidar', 'acc', 'acc_ot'])],
                   pub=[('/lead/odom_truth nav_msgs/Odometry', ['acc', 'acc_ot'])],
                   sub=[('/lead/cmd_vel geometry_msgs/Twist', ['acc', 'acc_ot'])]),
    'ekf': dict(pkg='robot_localization (ekf_sedan.yaml)', ekf_config='src/robotcar_localization/config/ekf_sedan.yaml',
                d='Fuses wheel velocity and IMU yaw rate into /odom_ekf and the odom to base_link TF. Every planning, safety and feature node reads /odom_ekf.',
                pub=[('/odom_ekf nav_msgs/Odometry', ['vlm', 'aeb', 'lidar', 'acc', 'acc_ot'])], sub=[]),
    'target': dict(pkg='scripts / tools/avkit',
                   d='The lap and scenario scripts (and the demo pane) publish a fixed /maneuver/target_speed at 10 Hz. Nothing moves without it: the MPC holds at 0 by design.',
                   pub=[('/maneuver/target_speed std_msgs/Float64', ['aeb', 'lidar', 'acc', 'acc_ot'])], sub=[]),
    'overtake_manager': dict(pkg='av_overtake_cpp (planned)',
                             d='Planned, not built. Sits between lane_node and local_planner through a launch remap: passes the lane path through unchanged, or shifts it into the left lane to pass a slow truck. Its speed cap is a required coordinator input.',
                             pub=[('/overtake/path_odom nav_msgs/Path', ['acc_ot']),
                                  ('/feature/overtake/constraint av_feature_msgs/VelocityConstraint', ['acc_ot']),
                                  ('/feature/overtake/status av_feature_msgs/OvertakeStatus', ['acc_ot'])],
                             sub=[('/lane/path_odom nav_msgs/Path', ['acc_ot']),
                                  ('/lidar/front/points sensor_msgs/PointCloud2', ['acc_ot']),
                                  ('/odom_ekf nav_msgs/Odometry', ['acc_ot'])]),
}

# launch remaps per node and setup (the launch file decides them in code); build.py checks
# every remap in av_stack.launch.py has a rule here
REMAPS = [
    ('mpc_tracker_v2', '/cmd_vel', '/control/mpc_cmd', ['aeb', 'lidar', 'acc', 'acc_ot']),
    ('mpc_tracker_v2', '/plan/path_odom', '/planning/authorized/path', ['acc', 'acc_ot']),
    ('mpc_tracker_v2', '/plan/speed_profile', '/planning/authorized/speed_profile', ['acc', 'acc_ot']),
    ('aeb_node', 'nominal_cmd', '/control/mpc_cmd', ['aeb', 'lidar', 'acc', 'acc_ot']),
    ('constraint_from_maneuver_node', 'constraint', '/feature/cruise/constraint', ['acc', 'acc_ot']),
    ('local_planner', '/lane/path_odom', '/overtake/path_odom', ['acc_ot']),  # planned
]

# placeholders the source cannot resolve by itself: (node, expression) -> topics, from a config file
PLACEHOLDERS = {
    ('trajectory_coordinator_node', 'topic'): dict(config='src/av_planning_coordinator/config/coordinator_lidar_acc.yaml',
                                                   key='constraint_topics', type='av_feature_msgs/VelocityConstraint'),
}
# topics a node only uses in some setups (chosen by a parameter at runtime, invisible to the scan)
ONLY_IN = {
    ('aeb_node', '/camera/front/depth/image_raw'): ['aeb'],                   # obstacle_source:=depth
    ('aeb_node', '/lidar/front/points'): ['lidar', 'acc', 'acc_ot'],          # obstacle_source:=lidar
}
EXTRA_SUBS = {  # planned inputs of built nodes
    'trajectory_coordinator_node': [('/feature/overtake/constraint av_feature_msgs/VelocityConstraint', ['acc_ot'])],
}

# drawn as one line with the subscriber list instead of one arrow each
BUS = {'/odom_ekf': dict(d='M375,655 L1105,655', lx=740, ly=655)}

TOPIC_LABEL = {
    '/camera/front/image_raw': 'RGB', '/camera/front/depth/image_raw': 'depth', '/camera/front/camera_info': None,
    '/lidar/front/points': 'lidar', '/odom_wheel_raw': 'wheel odom', '/imu': 'IMU', '/odom': '/odom',
    '/lane/path_odom': '/lane/path_odom', '/vlm/sign': '/vlm/sign', '/vlm/sign_roi': 'ROI',
}
TOPIC_KIND = {
    '/maneuver/target_speed': 'speed', '/maneuver/state': 'speed', '/vlm/sign': 'speed',
    '/feature/cruise/constraint': 'speed', '/feature/acc/constraint': 'speed', '/feature/overtake/constraint': 'speed',
    '/cmd_vel': 'cmd', '/control/mpc_cmd': 'cmd', '/control/aeb_cmd': 'cmd', '/safety/aeb_status': 'cmd',
    '/mpc/cmd_steer': 'fb', '/mpc_predicted_path': 'fb', '/vlm/sign_roi': 'fb', '/lead/cmd_vel': 'fb', '/lead/odom_truth': 'fb',
}

PORTS = [[100, 'RGB + depth'], [230, 'RGB + depth'], [320, 'lead truck'], [420, 'lidar 120 deg'],
         [520, 'depth / lidar'], [580, 'wheel odom'], [665, 'IMU'], [680, '/cmd_vel in']]

# 'from>to': SVG path, label position, optional fixed label ('' hides it) and kind
LOOP = 'L1120,705 L190,705 L190,680 L170,680'
ROUTES = {
    'gazebo>vlm_sign_node': dict(d='M170,60 L420,60', lx=300, ly=60),
    'gazebo>sign_maneuver_node': dict(d='M170,100 L640,100 L640,80 L680,80', lx=300, ly=100),
    'gazebo>lane_node': dict(d='M170,230 L420,230', lx=300, ly=230),
    'gazebo>acc_node': dict(d='M170,420 L420,420', lx=300, ly=420),
    'gazebo>aeb_node': dict(d='M170,520 L420,520', lx=300, ly=520),
    'gazebo>odom_noise': dict(d='M170,580 L215,580', label=''),
    'odom_noise>ekf': dict(d='M295,605 L295,630', lx=321, ly=618),
    'gazebo>ekf': dict(d='M170,665 L215,665', label=''),
    'lead_vehicle>gazebo': dict(d='M215,312 L170,312', label=''),
    'gazebo>lead_vehicle': dict(d='M170,334 L215,334', label=''),
    'vlm_sign_node>sign_maneuver_node': dict(d='M590,50 L680,50', lx=635, ly=50),
    'sign_maneuver_node>vlm_sign_node': dict(d='M680,66 L590,66', label=''),
    'lane_node>sign_maneuver_node': dict(d='M590,212 C640,212 630,74 680,74', label='', kind='fb'),
    'sign_maneuver_node>mpc_tracker_v2': dict(d='M850,60 L1020,60 L1020,200', lx=935, ly=60, label='target speed + state'),
    'target>mpc_tracker_v2': dict(d='M375,192 L980,192 L980,200', lx=520, ly=192),
    'target>constraint_from_maneuver_node': dict(d='M295,200 L295,330 L420,330', label=''),
    'lane_node>local_planner': dict(d='M590,235 L680,235', lx=635, ly=235),
    'local_planner>mpc_tracker_v2': dict(d='M850,235 L940,235', lx=895, ly=235, label='/plan/*'),
    'mpc_tracker_v2>lane_node': dict(d='M1060,200 L1060,181 L505,181 L505,200', lx=900, ly=181),
    'mpc_tracker_v2>gazebo': dict(d='M1105,245 L1120,245 ' + LOOP, lx=640, ly=705),
    'local_planner>trajectory_coordinator_node': dict(d='M765,260 L765,345', lx=795, ly=305, label='/plan/*'),
    'trajectory_coordinator_node>mpc_tracker_v2': dict(d='M850,360 C905,360 900,245 940,245', lx=900, ly=318, label='authorized plan'),
    'constraint_from_maneuver_node>trajectory_coordinator_node': dict(d='M590,330 C640,330 640,365 680,365', lx=634, ly=335, label='cruise cap'),
    'acc_node>trajectory_coordinator_node': dict(d='M590,420 C640,420 640,385 680,385', lx=634, ly=418, label='ACC cap'),
    'local_planner>acc_node': dict(d='M700,260 L700,290 L612,290 L612,440 L590,440', label='', kind='fb'),
    'mpc_tracker_v2>control_arbiter_node': dict(d='M1040,260 L1040,490', lx=1040, ly=430),
    'mpc_tracker_v2>aeb_node': dict(d='M970,260 L970,470 L505,470 L505,490', lx=780, ly=470, label='/mpc_predicted_path', kind='fb'),
    'aeb_node>control_arbiter_node': dict(d='M590,520 L940,520', lx=765, ly=520, label='brake cmd + AebStatus'),
    'control_arbiter_node>gazebo': dict(d='M1105,520 L1120,520 ' + LOOP, lx=640, ly=705),
    'lane_node>overtake_manager': dict(d='M590,218 C640,218 640,150 680,150', label=''),
    'overtake_manager>local_planner': dict(d='M765,175 L765,200', label=''),
    'gazebo>overtake_manager': dict(d='M170,280 L650,280 L650,165 L680,165', label=''),
    'overtake_manager>trajectory_coordinator_node': dict(d='M850,140 L872,140 L872,395 L850,395', lx=872, ly=290, label='overtake cap'),
}

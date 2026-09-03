`vehicle_model.{hpp,cpp}`, `path_utils.{hpp,cpp}`, `mpc_solver.{hpp,cpp}` are the
standalone solver library: no rclcpp dependency beyond message types, testable
offline via model_probe / mpc_probe .

Node-level behavior (speed-profile arbitration, stop-point arrival, command
shaping) lives in ../mpc_tracker_v2_node.cpp, NOT here. Keep it that way: the
solver stays a pure function from (state, reference, previous command) to
(accel, steer, prediction), which is what makes it testable without a sim.
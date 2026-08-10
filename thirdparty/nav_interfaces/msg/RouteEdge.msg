string id
nav_interfaces/RouteEndpoint from_ep
nav_interfaces/RouteEndpoint to_ep
string   edge_type       # "LINE" | "BEZIER"
bool     bidirectional
float32  max_speed
float32  xy_tolerance
float32  th_tolerance
float32  lane_width
string   obs_policy      # "STOP" | "DETOUR"
geometry_msgs/Point[] control_points

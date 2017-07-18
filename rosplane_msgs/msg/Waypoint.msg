# New waypoint, input to path manager

# @warning w and set_current always have to be valid; the chi_d is optional.
float32[3] w		# Waypoint in local NED (m)
float32 chi_d		# Desired course at this waypoint (rad)
bool chi_valid		# Desired course valid
float32 Va_d		# Desired airspeed (m/s)
bool set_current	# Sets this waypoint to be executed now!

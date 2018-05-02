# New waypoint, input to path manager

# @warning w and Va_d always have to be valid; the chi_d is optional.
float32[3] w		# Waypoint in local NED (m)
float32 Va_d		# Desired airspeed (m/s)
bool landing		# True if we want to land
bool set_current	# Sets this waypoint to be executed now! Starts a new list
bool clear_wp_list  # Removes all waypoints and returns to origin.  The rest of
                    # this message will be ignored

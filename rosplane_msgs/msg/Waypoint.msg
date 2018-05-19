# New waypoint, input to path manager

# @warning w and Va_d always have to be valid; the chi_d is optional.
float32[3] w		    # Waypoint in local NED (m)
float32 Va_d		    # Desired airspeed (m/s)
bool drop_bomb      # True if we want to drop the bomb near this waypoint (the waypoint is directly above the target location)
bool landing		    # True if we want to land
bool set_current	  # Sets this waypoint to be executed now! Starts a new list, clears all waypoints
bool clear_wp_list  # Removes all waypoints and returns to origin.  The rest of
                    # this message will be ignored
int8 priority       # priority of the waypoint. A higher number means more priority (5 is the highest)
                    # When sending a waypoint with a priority of say, 3, all waypoints in the que of priority less than 3 are deleted. (2, 1, 0)
                    # still use set_current to delete all waypoints and start with the new waypoints.
bool loiter_point   # true if we want to loiter about this point

# Controller commands output from the path follower or tuning node, input to autopilot controller

# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional
float32 Va_c		# Commanded airspeed (m/s)
float32 h_c			# Commanded altitude (m)
float32 chi_c		# Commanded course (rad)
float32 phi_ff		# feed forward command for orbits (rad)
uint8 aux_state		# State to determine how aux commands are interpreted
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid
bool landing			# True if we want to land

uint8 ROLL_PITCH_VA_TUNING = 0	# Tune roll loop, tune pitch, tune throttle to Va
uint8 COURSE_TUNING = 1			    # Tune course loop, hold pitch, hold Va


## aux_state         aux[0]        aux[1]  aux[2]  aux[3]
#
#      0      theta_c (rad)   phi_c (rad)      -       -
#      1      theta_c (rad)            -       -       -

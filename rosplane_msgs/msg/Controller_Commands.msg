# Controller commands output from the path follower or tuning node, input to autopilot controller

# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional
float32 Va_c		# Commanded airspeed (m/s)
float32 h_c			# Commanded altitude (m)
float32 chi_c		# Commanded course (rad)
float32 phi_ff		# feed forward command for orbits (rad)
uint8 aux_state		# State to determine how aux commands are interpreted
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid

uint8 ROLL_TUNING = 0			# Tune roll loop, tune pitch, feed through throttle
uint8 COURSE_TUNING = 1			# Tune course loop, tune pitch, feed through throttle
uint8 THR_VA_TUNING = 2			# Tune throttle to airspeed loop, tune pitch, stabilize roll
uint8 PITCH_VA_TUNING = 3		# Tune pitch to airspeed loop, feed through throttle, stabilize roll
uint8 PITCH_ALT_TUNING = 4		# Tune pitch to altitude loop, control airspeed with throttle, stabilize roll


## aux_state   aux[0]  aux[1]  aux[2]  aux[3]
# 
#      0      theta_c   phi_c   thr_c       -
#      1      theta_c       -   thr_c       -
#      2      theta_c   phi_c       -       -
#      3            -   phi_c   thr_c       -
#      4            -   phi_c       -       -

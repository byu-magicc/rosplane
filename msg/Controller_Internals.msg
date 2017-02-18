# Controller internals (inner loops, states) output from the autopilot controller, for now just for debuging and ploting

# @warning theta_c, phi_c and alt_zone have always to be valid, the aux array is optional
float32 theta_c		# Commanded pitch (rad)
float32 phi_c		# Commanded roll (rad)
uint8 alt_zone		# Zone in the altitude state machine
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid

uint8 ZONE_TAKE_OFF = 0
uint8 ZONE_CLIMB = 1
uint8 ZONE_DESEND = 2
uint8 ZONE_ALTITUDE_HOLD = 3

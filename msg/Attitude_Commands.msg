# Attitude commands output from the autopilot controller, for now just for debuging and ploting

# @warning theta_c and phi_c have always to be valid, the aux array is optional
float32 theta_c		# Commanded pitch (rad)
float32 phi_c		# Commanded roll (rad)
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid

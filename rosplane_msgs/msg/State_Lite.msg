# Vehicle state 'x_hat' pared down for low-bandwidth telemetry links 

Header header

# Original States
# @warning roll, pitch and yaw have always to be valid, the quaternion is optional
float32[3] position	# north, east, down (m)
float32 Va		# Airspeed (m/s)
float32 phi		# Roll angle (rad)
float32 theta		# Pitch angle (rad)
float32 chi		# Course angle (rad)

# Additional States for convenience
float32 initial_lat 	# Initial/origin latitude (lat. deg)
float32 initial_lon 	# Initial/origin longitude (lon. deg) 
float32 initial_alt 	# Initial/origin altitude (m) 

# RadarTrackObjectProjectSingle.msg
# Message definition for a single radar-tracked object with projection information

uint32 id  # Target ID

uint16 cluster_id  # ID of the radar target cluster

uint16 life  # Lifetime of the tracked object

uint16 associated_cnt  # Number of associations

uint16 lost_cnt  # Number of times the object was lost

float32 x  # X-coordinate, in meters (m)

float32 y  # Y-coordinate, in meters (m)

float32 z  # Z-coordinate, in meters (m)

float32 vx  # Velocity in the X direction, in meters per second (m/s)

float32 vy  # Velocity in the Y direction, in meters per second (m/s)

float32 x_size  # Object size in the X direction, in meters (m)

float32 y_size  # Object size in the Y direction, in meters (m)

float32 range  # Range, in meters (m)

float32 doppler  # Doppler shift, in meters per second (m/s)

float32 azimuth  # Azimuth angle, in degrees (deg)

float32 elevation  # Elevation angle, in degrees (deg)

float32 range_rate  # Range rate, in meters per second (m/s)

float32 unambvel  # Unambiguous velocity, in meters per second (m/s)

float32 rcs  # Radar cross-section, in decibels per square meter (dBm²)

float32 confidence  # Confidence level of the detection (0-1)

# Target type classification
# 0: Undefined
# 1: Small car
# 2: Large car
# 3: Motorcycle
# 4: Bicycle
# 5: Pedestrian
uint8 target_type  

# Motion state of the object
# 0: Static
# 1: Dynamic
# 2: Dynamic to Static
# 3: Static to Dynamic
# 4: Unknown state
uint8 motion_state  

# Event type (0-26)
# 0: Normal driving
# 1: Wrong-way driving
# 2: Large vehicle speeding
# 3: Small vehicle speeding
# 4: Large vehicle slow driving
# 5: Small vehicle slow driving
# 6: Parking violation
# 7: Occupying emergency lane
# 8: Illegal line press
# 9: Illegal lane change
# 10: Emergency lane parking violation
# 11: Emergency lane wrong-way driving
# 12: Pedestrian jaywalking
# 13: Animal break in
# 14: Scattering
# 15: Train runs on the main road.
# 16: NonVehicle
# 17: Crossing driversion
# 18: Parking driversion
# 19: Unsafe driving headway
# 20: Vehicle departure
# 21: Light congest
# 22: Medium congest
# 23: Serious congest
# 24: Rapid acceleration
# 25: Rapid deceleration
# 26: Quick turn
uint8 event  

uint8 lane_id  # Lane ID for the detected object

float32 meas_range  # Measured range to the object, in meters (m)

float32 meas_velocity  # Measured velocity, in meters per second (m/s)

float32 meas_sin_azimuth  # Sine of the azimuth angle, unitless

float32 meas_snr  # Signal-to-noise ratio of the measurement, in dB

float32 meas_peak_value  # Peak value of the measurement, in dB

float64 longitude  # Longitude of the target, in degrees (deg)

float64 latitude  # Latitude of the target, in degrees (deg)

# Normalized image bounding box coordinates
float32 x_pixel_norm_short  # x-coordinate in short camera image (normalized)
float32 y_pixel_norm_short  # y-coordinate in short camera image (normalized)
float32 x_pixel_norm_long   # x-coordinate in long camera image (normalized)
float32 y_pixel_norm_long   # y-coordinate in long camera image (normalized)

# World coordinates of the detected object
float64 x_world  # x-coordinate in world coordinate system (m)
float64 y_world  # y-coordinate in world coordinate system (m)
float64 z_world  # z-coordinate in world coordinate system (m)

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

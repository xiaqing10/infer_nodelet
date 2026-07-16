# RadarCloudObjectSingle.msg
# Message definition for a single radar object in a cloud

uint32 id  # Target ID

uint32 track_out_id  # ID of the tracking output (range 0-9999), 0 if no tracking data available

uint16 range_idx  # Range bin index (distance measurement)

uint16 doppler_idx  # Doppler bin index (velocity measurement)

float32 range  # Range, in meters (m)

float32 velocity  # Velocity, in meters per second (m/s)

float32 sin_azimuth  # Sine of the azimuth angle

float32 azimuth  # Azimuth angle, in degrees (deg)

float32 elevation  # Elevation angle, in degrees (deg)

float32 x  # X-coordinate, in meters (m)

float32 y  # Y-coordinate, in meters (m)

float32 z  # Z-coordinate, in meters (m)

float32 peak_value  # Peak value, in dB

float32 doppler_snr  # Doppler signal-to-noise ratio, in dB

float32 angle_snr  # Angle signal-to-noise ratio, in dB

float32 snr  # Overall signal-to-noise ratio, in dB

float32 unambvel  # Unambiguous velocity, in meters per second (m/s)

uint8 moving_flag  # Moving flag: 0 = stationary, 1 = dynamic


uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

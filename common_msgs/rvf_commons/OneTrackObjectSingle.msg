# OneTrackObjectSingle.msg
# Message definition for a single tracked object in a system

uint32 id  # Target ID

float64 start_time_stamp  # Start time of detection, in seconds (s)

uint16 start_radar_index  # Radar index that started the tracking

uint32 start_track_id     # ID of the start tracking process

uint16 associated_radar_index  # Associated radar index for this detection

uint32 associated_track_id  # Associated track ID

uint16 associated_camera_index  # Associated camera ID for this detection

float32 x  # X-coordinate of the target, in meters (m)

float32 y  # Y-coordinate of the target, in meters (m)

float32 z  # Z-coordinate of the target, in meters (m)

float32 vx  # Velocity in the X direction, in meters per second (m/s)

float32 vy  # Velocity in the Y direction, in meters per second (m/s)

float32 x_size  # Size of the target in the X direction, in meters (m)

float32 y_size  # Size of the target in the Y direction, in meters (m)

# Target type classification (0-36)
# 0: Undefined  
# 1: Dense Crowd  
# 2: Person  
# 3: Small Car  
# 4: Van  
# 5: Bus  
# 6: Truck  
# 7: Motorcycle  
# 8: Bicycle  
# 9: Tricycle with Sunshade  
# 10: Tricycle  
# 11: Animal  
# 21: Box  
# 22: Tire  
# 23: Tree Branch  
# 31: Construction Vehicle  
# 32: Construction Worker  
# 33: Cone  
# 34: Pillar Cone  
# 35: Roadblock  
# 36: Road Sign
uint8 target_type 

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

# Direction of travel (0: Outside the lane, 1: Going, 2: Coming)
uint8 direction  

# Vehicle color
# 0: Undefined
# 1: White
# 2: Black
# 3: Red
# 4: Yellow
# 5: Gray
# 6: Blue
# 7: Green
# 8: Brown
uint8 color  

string plate_number  # License plate number

float32 plate_confid  # Confidence score for the plate detection, range [0,1]

# Plate color
# 0: Undefined
# 1: Blue
# 2: Green
# 3: Yellow
# 4: White
# 5: Black
uint8 plate_color  

float64 x_world  # X-coordinate in world coordinate system, in meters (m)

float64 y_world  # Y-coordinate in world coordinate system, in meters (m)

float64 z_world  # Z-coordinate in world coordinate system, in meters (m)

float64 longitude  # Longitude of the target, in degrees (deg)

float64 latitude  # Latitude of the target, in degrees (deg)

float32 x_pixel_norm  # Normalized x-coordinate of the bounding box

float32 y_pixel_norm  # Normalized y-coordinate of the bounding box

float32 w_pixel_norm  # Normalized width of the bounding box

float32 h_pixel_norm  # Normalized height of the bounding box

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

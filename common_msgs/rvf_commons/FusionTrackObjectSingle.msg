# FusionTrackObjectSingle.msg
# Message definition for a single fusion-tracked object

uint32 id  # Target ID

float64 start_time_stamp  # Timestamp of object initialization, in seconds (s)

uint8 fusion_flag  # Fusion status: 0 = Not fused, 1 = Fused

# Object position and velocity
float32 x  # X-coordinate, in meters (m)
float32 y  # Y-coordinate, in meters (m)
float32 z  # Z-coordinate, in meters (m)
float32 vx  # Velocity in the X direction, in meters per second (m/s)
float32 vy  # Velocity in the Y direction, in meters per second (m/s)
float32 ax  # Acceleration in the X direction, in meters per second squared (m/s²)
float32 ay  # Acceleration in the Y direction, in meters per second squared (m/s²)
float32 yaw  # Yaw angle, in degrees (°), base to north direction, [-360 360]
float32 yaw_rate  # Yaw rate, in degrees per second (°/s)

# Object size
float32 x_size  # Object size in the X direction, in meters (m)
float32 y_size  # Object size in the Y direction, in meters (m)

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

# Target direction (0-2)
# 0: Unknown
# 1: Forward
# 2: Reverse
uint8 direction  


# Vehicle color classification (0-8)
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
float32 plate_confid  # License plate confidence score, range [0,1]

# License plate color classification (0-5)
# 0: Undefined
# 1: Blue 
# 2: Green 
# 3: Yellow 
# 4: White 
# 5: Black 
uint8 plate_color  

# World coordinates of the detected object
float64 x_world  # X-coordinate in world coordinate system, in meters (m)
float64 y_world  # Y-coordinate in world coordinate system, in meters (m)
float64 z_world  # Z-coordinate in world coordinate system, in meters (m)

# Geographical location of the detected object
float64 longitude  # Longitude of the target, in degrees (deg)
float64 latitude  # Latitude of the target, in degrees (deg)

string pole_name  # Pole name (as a string)
uint16 pole_index  # Pole index (numbering starts from 1)

# Radar tracking information
uint16 start_radar_index       # Radar ID where tracking started
uint16 associated_radar_index  # Associated radar index
uint32 associated_track_id    # Associated tracking ID

# Target classification from radar perspective (0-5)
# 0: Undefined
# 1: Small car
# 2: Large car
# 3: Motorcycle
# 4: Bicycle
# 5: Pedestrian
uint8 target_type_radar  

float32 x_radar  # X-coordinate in radar coordinate system, in meters (m)
float32 y_radar  # Y-coordinate in radar coordinate system, in meters (m)
float32 rcs_radar  # Radar cross-section (RCS), in decibels per square meter (dBm²)

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
uint8 event_radar  

uint8 land_id_radar  # Lane ID from radar

uint16 associated_camera_index  # Associated camera index

# Target type classification (0-10)
# 0: Undefined
# 1: Dense crowd
# 2: Person
# 3: Car
# 4: Van
# 5: Bus
# 6: Truck
# 7: Motorcycle
# 8: Bicycle
# 9: Canopied tricycle
# 10: Tricycle  
uint8 target_type_camera  

# Normalized image bounding box coordinates
float32 x_pixel_norm  # Normalized x-coordinate of the top-left corner
float32 y_pixel_norm  # Normalized y-coordinate of the top-left corner
float32 w_pixel_norm  # Normalized bounding box width
float32 h_pixel_norm  # Normalized bounding box height

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

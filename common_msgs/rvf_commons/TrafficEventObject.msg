# TrafficEventObject.msg
# Message definition for traffic event objects

Header header  # Header (includes timestamp and frame ID)

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

# bitmap
# bit1       | bit2             | bit 3         | bit 4           | bit 5       | bit 6      | bit 7     | bit 8  
# retrograde   overSpeed         lowSpeed         illegalStop      emergencyLane  laneChange   linePress   breakIn
# bit9       | bit10            | bit 11        | bit 12          | bit 13      | bit 14     | bit 15 
# nonVehicle   crossDriversion   parkDriversion   nonSafeHeadway   rapidAcc       rapidDec     quickTurn 
uint32 special_event

# 0: Out lane
# 1: Carriageway
# 2: Overtaking lane
# 3: Emergency lane
# 8: Ramp
# 9: Diversion Line Area
uint8 lane_type

# Event source (1 = Fusion, 2 = Camera)
# 1: Fusion
# 2: Camera
uint8 event_src  

uint32 id  # Target ID (unique identifier for the event)

float32 x  # X-coordinate of the target, in meters (m)
float32 y  # Y-coordinate of the target, in meters (m)
float32 z  # Z-coordinate of the target, in meters (m)
float32 vx  # Velocity in the X direction, in meters per second (m/s)
float32 vy  # Velocity in the Y direction, in meters per second (m/s)

float32 azimuth  # Azimuth angle, in degrees (deg)
float32 elevation  # Elevation angle, in degrees (deg)

float32 x_pixel_norm  # Normalized x-coordinate of the top-left corner of the bounding box
float32 y_pixel_norm  # Normalized y-coordinate of the top-left corner of the bounding box
float32 w_pixel_norm  # Normalized width of the bounding box
float32 h_pixel_norm  # Normalized height of the bounding box

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

uint8 lane_direction  # Lane direction (0: Outside the lane, 1: Upbound, 2: Downbound)

uint8 direction  # Direction of travel (0: Outside the lane, 1: Going, 2: Coming)

int16 lane_id  # Lane ID (positive for driving lanes, negative for non-driving lanes)

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
float32 plate_confid  # License plate confidence score, range [0,1]

# License plate color classification (0-5)
# 0: Undefined
# 1: Blue 
# 2: Green 
# 3: Yellow 
# 4: White 
# 5: Black 
uint8 plate_color  

string pole_name  # Name of the pole (or object) from which the event was detected
uint16 pole_index  # Index of the pole (first pole, second pole, etc.)

# Scene type (0: normal, 1: Main road, 2: Tunnel)
uint8 scene  

float64 longitude  # Longitude of the target, in degrees (deg)

float64 latitude  # Latitude of the target, in degrees (deg)

uint16 camera_index  # Camera index (1 = Camera 1, 2 = Camera 2, etc.)
string video_file_path  # Path to the video file for the event
string image_file_path  # Path to the image file for the event
string radar_cloud_file_path  # Path to the radar data file for the event
string radar_track_file_path  # Path to the radar track file for the event
string fusion_track_file_path  # Path to the fusion track file for the event

uint8[] reserved1  # Reserved field
string reserved2  # Reserved field

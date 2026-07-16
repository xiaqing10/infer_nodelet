# ImageDetectObjectProjectSingle.msg
# Message definition for a single detected object with projected coordinates

uint32 id  # Target ID

# Normalized bounding box coordinates of the detected object
float32 x_pixel_norm  # Top-left corner x-coordinate, normalized to [0,1]
float32 y_pixel_norm  # Top-left corner y-coordinate, normalized to [0,1]
float32 w_pixel_norm  # Bounding box width, normalized to [0,1]
float32 h_pixel_norm  # Bounding box height, normalized to [0,1]

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

# Coordinates relative to radar detection
float32 x_radar  # x-coordinate of the target relative to the detected radar position (m)
float32 y_radar  # y-coordinate of the target relative to the detected radar position (m)

# World coordinates of the detected object
float64 x_world  # x-coordinate in world coordinate system (m)
float64 y_world  # y-coordinate in world coordinate system (m)
float64 z_world  # z-coordinate in world coordinate system (m)

# Geographical location of the detected object
float64 longitude  # Longitude of the target (degrees)
float64 latitude  # Latitude of the target (degrees)

uint8[] reserved1  # Reserved field
string reserved2  # Reserved field

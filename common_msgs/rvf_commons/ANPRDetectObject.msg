# BayonetDetectObject.msg
# Message definition for bayonet detection of vehicles

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint64 arrive_tick  # Arrival timestamp, in milliseconds (ms)

uint8 lane_id  # Lane ID

string plate_number  # License plate number

# Vehicle plate color
# 0: Undefined
# 1: Blue
# 2: Green
# 3: Yellow
# 4: White
# 5: Black
uint8 plate_color  

float32 plate_confid  # License plate confidence score, range [0,1]

string car_color  # Vehicle color as detected by the system

string car_type  # Vehicle type as detected by the system

string car_brand  # Vehicle brand as detected by the system

float64 longitude  # Longitude, in degrees (deg)

float64 latitude  # Latitude, in degrees (deg)

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

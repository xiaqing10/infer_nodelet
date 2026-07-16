# TrafficFlowSingle.msg
# Message definition for traffic flow statistics for a single lane

uint8 lane_id  # Lane ID

# Lane direction 
# 0: Outside the lane
# 1: Upbound
# 2: Downbound
uint8 lane_direction  

# Direction of travel (0: Outside the lane, 1: Going, 2: Coming)
uint8 direction  

uint32 flow  # Flow of vehicles in the lane

uint32 flow_person  # Flow of pedestrians

uint32 flow_car  # Flow of cars

uint32 flow_truck  # Flow of trucks

uint32[] flow_reserved  # Reserved flow data

float32 average_speed  # Average speed of vehicles in the lane

float32 average_speed_person  # Average speed of pedestrians, in meters per second (m/s)

float32 average_speed_car  # Average speed of cars, in meters per second (m/s)

float32 average_speed_truck  # Average speed of trucks, in meters per second (m/s)

float32[] average_speed_reserved  # Reserved average speed data, in meters per second (m/s)

float32 time_occupancy  # Time occupancy

float32 time_headway # Time headway

float32 space_occupancy  # Space occupancy

float32 space_headway  # Space headway

float32 eightyfive_bit_speed  # 85th percentile speed, in meters per second (m/s)

# Degree of congestion
# 0: No congestion
# 1: Light congestion
# 2: Medium congestion
# 3: Severe congestion
uint8 congestion_degree  

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

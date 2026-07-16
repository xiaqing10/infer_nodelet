# TrafficFlow.msg
# Message definition for traffic flow statistics across multiple lanes

float64 time_stamp_start  # Start time of the traffic flow, in seconds (s)

float64 time_stamp_end  # End time of the traffic flow, in seconds (s)

string pole_name  # Name of the pole (for identification)

uint16 pole_index  # Index of the pole (first pole, second pole, etc.)

uint16 radar_index # Index of the radar (first radar, second radar, etc.)

uint32 total_flow # Total flow of all target

uint32 total_flow_person  # Total flow of pedestrians

uint32 total_flow_car  # Total flow of cars

uint32 total_flow_truck  # Total flow of trucks

uint32[] total_flow_reserved  # Reserved flow data

float32 total_average_speed # Average speed of all target, in meters per second (m/s)

float32 total_average_speed_person  # Average speed of pedestrians, in meters per second (m/s)

float32 total_average_speed_car  # Average speed of cars, in meters per second (m/s)

float32 total_average_speed_truck  # Average speed of trucks, in meters per second (m/s)

float32[] total_average_speed_reserved  # Reserved average speed data, in meters per second (m/s)

float32 total_time_occupancy # Total time occupancy

float32 total_time_headway  # Total time headway

float32 total_space_occupancy  # Total space occupancy

float32 total_space_headway  # Total space headway

float32 total_eightyfive_bit_speed  # Total 85th percentile speed, in meters per second (m/s)

uint8 lane_number  # Number of lanes

TrafficFlowSingle[] data  # Array of traffic flow data for individual lanes

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

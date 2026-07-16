# FusionTrackStatiSingle.msg
# Message definition for fusion tracking statistics of a single object

string pole_name    # Name of ploe

uint8 radar_number  # Number of radar devices involved in tracking

uint8 camera_number  # Number of camera devices involved in tracking

uint16[] radar_track_number  # Number of radar tracks involved

uint16[] camera_track_number  # Number of camera tracks involved

uint16[] fusion_track_number # Number of fusion tracks involved

float32[] fusion_rate  # Fusion rate (percentage of fused detections)

float32[] fusion_x_std  # Standard deviation in the X direction of fused data

float32[] fusion_y_std  # Standard deviation in the Y direction of fused data

float32[] fusion_angle_bias  # Angle bias of the fused data

float32[] fusion_time_diff  # Time difference of the fused data

uint8 section_number  # Number of sections in the tracked area

float32 section_range # Range of a section

float32[] section_fusion_rate  # Fusion rate for each section

float32[] section_fusion_x_std  # Standard deviation in the X direction for each section

float32[] section_fusion_y_std  # Standard deviation in the Y direction for each section

float32[] section_fusion_angle_bias  # Angle bias for each section

uint32 total_points_number  # Total number of points tracked

float32 average_fusion_rate  # Average fusion rate across all sections

float32 average_fusion_x_std  # Average standard deviation in the X direction

float32 average_fusion_y_std  # Average standard deviation in the Y direction

float32 average_fusion_angle_bias  # Average angle bias

float32[] horizontal_mounting_angle  # Horizontal mounting inclination angle(degree)

float32[] pitch_mounting_angle  # Pitch mounting inclination angle(degree)

float32[] radar_mounting_height  # radar mounting height(m)


uint8[] reserved1  # Reserved field

string reserved2  # Reserved field


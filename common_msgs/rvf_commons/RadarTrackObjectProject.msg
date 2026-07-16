# RadarTrackObjectProject.msg
# Message definition for radar-tracked objects with projection information

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of detected tracked objects

RadarTrackObjectProjectSingle[] objects  # Array of tracked radar objects with projection information

float32[] project_radar_to_camera  # Projection matrix from radar to camera, used for transforming radar coordinates to camera coordinates

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

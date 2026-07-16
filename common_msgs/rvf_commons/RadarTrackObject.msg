# RadarTrackObject.msg
# Message definition for radar-tracked objects

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of detected tracked objects

RadarTrackObjectSingle[] objects  # Array of tracked radar objects

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

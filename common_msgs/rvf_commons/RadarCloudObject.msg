# RadarCloudObject.msg
# Message definition for radar cloud objects

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of detected radar cloud objects

RadarCloudObjectSingle[] objects  # Array of detected radar cloud objects

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

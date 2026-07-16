# FusionTrackObject.msg
# Message definition for fused tracked objects

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of fused tracked objects

FusionTrackObjectSingle[] objects  # Array of fused tracked objects

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

# OneTrackObject.msg
# Message definition for a list of tracked objects

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of tracked objects

OneTrackObjectSingle[] objects  # Array of tracked objects

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

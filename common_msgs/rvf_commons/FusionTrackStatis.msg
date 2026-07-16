# FusionTrackStatis.msg
# Message definition for fusion tracking statistics of multiple objects

Header header  # Header (includes timestamp and frame ID)

uint8 pole_number  # Pole number (for MEC-specific data)

FusionTrackStatisSingle[] data  # Array of fusion tracking statistics for each object

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

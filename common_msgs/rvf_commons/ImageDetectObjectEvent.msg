# ImageDetectObjectEvent.msg
# Message definition for detected objects with event information

Header header  # Header (includes timestamp and frame ID)

uint16 frame_seq  # Frame sequence number, increments sequentially (1, 2, 3, ...)

uint16 objects_number  # Number of detected objects

ImageDetectObjectEventSingle[] objects  # Array of detected objects with event information

float32[] project_camera_to_radar  # Projection matrix from camera to radar, used for transforming camera coordinates to radar coordinates

float32[] view_offset  # View offset in pixels (x, y) for the detected objects

uint8[] reserved1  # Reserved field

string reserved2  # Reserved field

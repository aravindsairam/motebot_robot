# One no-go zone of a saved map: a place the robot must never drive into, in the map frame (metres).
string name                      # unique among the map's zones; at most 40 characters
geometry_msgs/Polygon polygon    # 3 to 20 corners, in order around the outline; z is not used

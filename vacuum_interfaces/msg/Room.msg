# One room of a saved map, in the map frame (metres). Rooms are found from the map (split at
# doorways) and named by the user or by the robot looking at them.
string name                      # unique among the map's rooms; at most 40 characters
geometry_msgs/Polygon outline    # 3 or more corners, in order around the room's floor; z is not used
float64 x                        # a good place to go to in the room: its most open spot
float64 y
float64 area                     # m2 of floor

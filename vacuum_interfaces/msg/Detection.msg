# One thing the object finder saw in a camera picture.
string label            # what was asked for and found ("sofa")
float32 score           # 0 to 1: how sure the finder is
float32[4] box          # in the picture, pixels: x0, y0, x1, y1
bool located            # the depth camera gave its distance, so x, y are known
float64 x               # where it is on the map (metres): the near side of it, at floor level
float64 y
float32 distance        # m from the robot
float32 bearing         # radians, from the robot's heading, left positive

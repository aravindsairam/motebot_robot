# What to clean and how: the request of /cleaning/start and /cleaning/preview.

uint8 EVERYTHING = 0   # all the floor the robot can reach from where it is
uint8 AREA = 1         # inside the polygon `area`
uint8 SPOT = 2         # a square of the robot's spot_size (1.5 m) around (spot_x, spot_y)
uint8 PLAN = 3         # a saved plan, by name: its area, rows and options

uint8 OPTION_DEFAULT = 0
uint8 OPTION_OFF = 1
uint8 OPTION_ON = 2

uint8 what
float64[] area         # AREA: corners x0, y0, x1, y1, ... in the map frame, metres
float64 spot_x         # SPOT: the point, map frame, metres
float64 spot_y
string plan_name       # PLAN
string direction       # rows along "x" or "y"; "" or "auto": whichever needs fewer turns
uint8 passes           # 1, or 2 for a second pass at right angles to the first; 0 means 1
uint8 edges            # OPTION_*: a pass along walls and furniture after the rows (default: the robot's setting)
uint8 return_to_dock   # OPTION_*: dock when finished (default: on)

# The settings a user may change from the app: /robot_settings (latched), published by the mode
# manager, which keeps them in <maps dir>/robot_settings.yaml and changes them through
# /set_robot_settings. The cleaning node and the gateway follow this topic.

# What is accepted. Lengths in metres, speeds in m/s.
float32 CLEANING_SPEED_MIN = 0.10
float32 CLEANING_SPEED_MAX = 0.30
float32 MANUAL_SPEED_MIN = 0.10
float32 MANUAL_SPEED_MAX = 0.30
float32 ROW_SPACING_MIN = 0.10
float32 ROW_SPACING_MAX = 0.30    # the cleaning width: wider rows would leave strips of floor
float32 ROW_SPACING_STEP = 0.05   # the spacing is a whole number of these (one map cell)
float32 MANUAL_TURN_PER_SPEED = 4.0  # rad/s of turning at full joystick, per m/s of manual_speed

float32 cleaning_speed    # on the rows and edges of a clean; a clean under way changes speed at once
float32 manual_speed      # with the joystick pushed all the way; turning is scaled with it
float32 row_spacing       # between cleaning rows; applies from the next plan
bool row_angle_auto       # true: in each room, whichever way needs fewer turns
float32 row_angle         # degrees from the map's x axis, 0 up to (not including) 180; used when not auto

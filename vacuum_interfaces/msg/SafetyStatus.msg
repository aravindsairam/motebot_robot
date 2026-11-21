# What the safety gate is doing, published on /safety_status.
# The gate is the only node that commands the wheels. It decides who may drive from the robot's
# mode, stops the robot when commands go quiet, and reacts to the cliff sensors and the bumper.

string SOURCE_NONE = "none"
string SOURCE_MANUAL = "manual"     # keyboard or app joystick, on /cmd_vel
string SOURCE_NAV = "nav2"          # Nav2, on /cmd_vel_nav

builtin_interfaces/Time stamp
string source             # who is allowed to drive right now
string blocked_reason     # why the command is being overridden or held at zero; empty when passed through
bool cliff                # a cliff sensor sees no floor
bool bumper               # the bumper is pressed
bool off_floor            # every cliff sensor sees no floor: the robot was lifted or has fallen
bool reflex_active        # the gate is backing the robot away from a cliff or a bump
string[] stale            # inputs the gate has stopped receiving: "mode", "cliff", "bumper"

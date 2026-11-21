# What the robot is doing, published by the mode manager on /robot_state.

# The modes. More are added as features arrive (charging).
string IDLE = "idle"              # stopped, brake held, waiting for a request
string MAPPING = "mapping"        # building a new map while someone drives
string NAVIGATION = "navigation"  # localized on a saved map, Nav2 accepts goals
string MANUAL = "manual"          # someone drives; localized if a map exists
string CLEANING = "cleaning"      # a cleaning job runs or is paused (the cleaning node drives through Nav2)
string EXPLORING = "exploring"    # the robot maps by itself: the map keeps growing while Nav2 drives. Entered
                                  # from mapping, only by an exploration; ends in mapping, with the map kept
string DOCKING = "docking"        # driving onto the dock; becomes docked, or idle if it fails
string DOCKED = "docked"          # parked on the dock
string FAILSAFE = "failsafe"      # something is wrong: stopped until the fault clears and idle is requested

builtin_interfaces/Time stamp     # when this state was entered
string state
string previous_state
string reason                     # why the last change happened: a request, or the fault that caused it
bool transitioning                # true while a mode change is in progress
string[] allowed_transitions      # states that can be requested right now
string[] faults                   # problems detected right now; empty when healthy
string current_map                # name of the saved map in use, empty if none
bool making_map                   # a map is being made: true in mapping and exploring, and in a failsafe
                                  # entered from either while the map is still there. Ask this, not the
                                  # mode's name, for "would leaving lose the map?" and "is there a saved
                                  # map to work on?" (current_map then still names the older saved map)

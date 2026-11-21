# How the exploration is going, published by the exploration node on /exploration/status
# (latched, on change and five times a second, job or not). An exploration is the robot mapping
# by itself: it drives to the edges of what it has seen until nothing is left to look at, comes
# back to where it started, and ends in mapping mode with the map kept, ready to be saved.
# The mode manager watches this in exploring mode: silent for 1 s, or saying there is no job
# (none, finished, stopped, failed), means failsafe.

string NONE = "none"             # no exploration since the exploration node started
string STARTING = "starting"     # accepted; switching to mapping and then to exploring mode (the only
                                 # time the mode manager allows exploring mode to be entered)
string EXPLORING = "exploring"   # the explorer is choosing places to look at and driving to them
string RETURNING = "returning"   # nothing is left to look at; driving back to where it started
string ENDING = "ending"         # over; handing back to mapping mode
string FINISHED = "finished"     # the last exploration ended by itself; `message` says how
string STOPPED = "stopped"       # the last exploration was stopped (Stop, or another mode was asked for)
string FAILED = "failed"         # the last exploration could not go on; `message` says why

builtin_interfaces/Time stamp
string state
string message                   # what it is doing, or how and why it ended
uint32 job_id                    # 1, 2, 3, ... since the exploration node started; 0 before the first
float64 started                  # when the exploration started, seconds since 1970; 0 before the first
uint32 places_left               # places still to look at (frontiers the explorer has not given up on)
float32 mapped_area              # m2 of floor on the map so far
float32 exploring_time           # s of robot time since the exploration started
float32 time_limit               # s of robot time after which the exploration ends early
float32 distance                 # m driven
uint32 goals_sent                # places the explorer has sent the robot to
uint32 goals_failed              # of those, the ones Nav2 refused or could not reach

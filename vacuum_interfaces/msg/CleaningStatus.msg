# How the cleaning job is going, published by the cleaning node on /cleaning/status (latched,
# on change and five times a second, job or not). Every number comes from where the robot has
# really been, not from the plan. The mode manager watches it in cleaning mode: silent for 1 s,
# or saying there is no job (none, finished, stopped, failed), means failsafe.

string NONE = "none"           # no job since the cleaning node started
string STARTING = "starting"   # accepted and planned; switching to cleaning mode (the only time the
                               # mode manager allows cleaning mode to be entered)
string CLEANING = "cleaning"
string PAUSED = "paused"       # standing still in cleaning mode; resume continues
string ENDING = "ending"       # the job is over; handing over to the next mode (docking or idle)
string FINISHED = "finished"   # the last job ended: everything planned was driven
string STOPPED = "stopped"     # the last job was stopped (Stop, Dock, another mode, cancelled)
string FAILED = "failed"       # the last job could not go on; `message` says why

builtin_interfaces/Time stamp
string state
string message                 # "row 12 of 31", or why it stopped or failed
uint32 job_id                  # 1, 2, 3, ... since the cleaning node started; 0 before the first
string target                  # "everything", "area", "spot", or the saved plan's name
CleaningJob job
float64 started                # when the job started, seconds since 1970; 0 before the first
float32 coverage               # percent of the planned floor cleaned so far
float32 cleaned_area           # m2 of floor cleaned so far
float32 planned_area           # m2 of floor the plan passes over
float32 distance               # m driven
float32 cleaning_time          # s, paused time not counted
float32 time_left              # s, estimated
uint32 rows_done
uint32 rows_total
uint32 rows_skipped            # blocked rows given up on
uint32 current_row             # the row being driven or the next one; 0 when no rows are left
uint32 edges_done
uint32 edges_total
uint32 edges_skipped
uint32 current_edge
uint32 piece                   # index in /cleaning/plan's pieces of the piece being driven; those before it are done
uint32 bumps                   # bumper presses while cleaning
uint32 pauses

# What a cleaning plan will take, worked out before the robot moves.
uint32 rows
uint32 edges               # straight stretches of the pass along the edges
uint32 turns
uint32 transits            # trips Nav2 plans between parts
float32 distance           # m, transits estimated
float32 planned_area       # m2 of floor the plan passes over
float32 left_out_area      # m2 the plan could reach but leaves (row ends, bits too small for a row)
float32 unreachable_area   # m2 in parts the robot cannot get to from where it is
float32 time               # s, expected
float32 planning_time      # s it took to plan

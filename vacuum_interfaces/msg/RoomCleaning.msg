# How clean one room is, from the cleaning records: its part of /cleaning/room_report.
string name
float64 floor_area            # m2 of floor in the room
float64 last_cleaned          # when the last clean that covered most of it (at least half) ended, seconds since 1970; 0: none recorded
float32 last_share            # percent of the room's floor that clean covered
float64 last_touched          # when any clean last passed through it (at least 5 % of its floor), seconds since 1970; 0: none
uint32 cleans_last_week       # cleans in the last 7 days that covered at least half of it

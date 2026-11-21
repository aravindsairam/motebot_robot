# The rows of the cleaning job running, or of the last preview. Published by the cleaning node on
# /cleaning/plan (latched). Empty when there is neither.
std_msgs/Header header        # frame "map"
uint32 job_id                 # the job driving it; 0 for a preview
string target                 # "everything", "area", "spot", or a saved plan's name
CleaningPiece[] pieces        # in driving order
CleaningEstimate estimate

# A saved cleaning plan, as listed by /cleaning/list_plans.
string name
CleaningJob job               # what it cleans and its options
CleaningEstimate estimate     # as it was when saved
string map_name
bool map_changed              # the map has changed since it was saved: its rows will be planned again
float64 saved                 # seconds since 1970

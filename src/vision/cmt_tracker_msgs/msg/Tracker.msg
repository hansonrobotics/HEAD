Header header
Object object # this holds face ovelaps nad emotime overlaps.

geometry_msgs/Point point

std_msgs/Int32 initial_points
std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.

std_msgs/Bool quality_results # Makes sense of the quality of the tracker.

std_msgs/String tracker_name #This is needed. Let this be a string:
std_msgs/String recognized_name #This is assigned when it's known to have a name
std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.

std_msgs/Int32 before_demotion
std_msgs/Bool validated #To check for merge fucntions.

std_msgs/String google_best_guess

Header header
Object object
std_msgs/Int32 initial_points
std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.
std_msgs/Bool quality_results # Makes sense
std_msgs/String tracker_name #This is needed.
std_msgs/String recognized_name #This is assigned when it's known to have a name
std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.
std_msgs/Int32 before_demotion
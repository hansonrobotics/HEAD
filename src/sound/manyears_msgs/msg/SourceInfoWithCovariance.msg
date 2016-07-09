# A source info message with a 3D-localized source with covariance.
# Replaces the point + latitude and longitude info of a standard SourceInfo
# message.

uint32 source_id
geometry_msgs/PoseWithCovarianceStamped pose
float32 source_probability
float32[] separation_data # Separation data (audio stream)
float32[] postfiltered_data # Postfiltered data (audio stream)


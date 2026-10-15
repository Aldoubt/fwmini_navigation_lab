# Pose of the selected tag in header.frame_id at the original image timestamp.
# A missing/rejected target emits valid=false, never a fabricated pose.
std_msgs/Header header
int32 id
string family
geometry_msgs/Pose pose
float32 decision_margin
int32 hamming
bool valid
string reason

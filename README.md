Perception Synthesizer
======================

Planed function: accept video (pi_vision) from multple sources, convert 
to 3D spatial coordinates using ros-tf. Publish intersting events to all
concerned. Interesting events include:
*Entry, exit of a human face in the field of view.
*Location of the face.

The goal here is to make sure that the behavior nodes have access to this
information, rather than having it drive the facial responses directly.

### Nodes
#### faces_tf2_broadcaster
Broadcast `/tf` data from face tracking.

###### Subscribed topics
 * `/faces3d (pi_vision/Faces)`: Faces published by pi_vision in 3D coordinates

###### Testing
 * Make sure the face tracking is publishing faces
 * Run the node: `rosrun perception faces_tf2_broadcaster` 
 * Open rviz: ``rosrun rviz rviz -d `rospack find perception`/rviz/faces.rviz``

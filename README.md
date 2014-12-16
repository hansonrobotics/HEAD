Perception Synthesizer
======================

Currently, this does nothing at all.

Planed function: accept video (pi_vision) from multple sources, convert 
to 3D spatial coordinates using ros-tf. Publish intersting events to all
concerned. Interesting events include:
*Entry, exit of a human face in the field of view.
*Location of the face.

The goal here is to make sure that the behavior nodes have access to this
information, rather than having it drive the facial responses directly.

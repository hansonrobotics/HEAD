# Implementation internals

There are two parts of code. 

First part is the part which listens to the Faceshift Network streaming using the faceshift network interface by using specified IP and port and publishes them to the /blender_api/set_shape_keys. This is launched via

    roslaunch faceshift_puppetering faceshift_puppetering.launch

This runs on localhost on 33433 port. If there is no element is terminates. If one desires to listen to a remote faceshift instance. 

    roslaunch faceshift_puppetering faceshift_puppetering.launch IP:=ip_host Port:=port_num
    
This node is entirelly independent of the the type of shapekeys being sent from the faceshift. It just prints key value pair of the name of shapekey with it's value. 


The second one are the internal changes to the blender_api to listen to this published topics 

    rostopic pub /blender_api/set_animation_mode blender_api_msgs/AnimationMode "value: 1" --once  # to set the mode as normal. 
    
    rostopic pub /blender_api/set_animation_mode blender_api_msgs/AnimationMode "value: 0" --once # to set the mode as faceshift led pupeetering. 

When animation mode is set to '1' the AnimationManager 
1. First saves all the drivers of the shapekeys that are going to be driven by faceshift (found by reading the JSON file) by iterating through them. 
2. Delete all the drivers of elements found in the keyshape pair json file.
3. Sets the value of the shapekeys to the value of the faceshift value by adding them or subtracting them as they are specified. Currently we just mapped the shapekeys of blender to faceshift one to one. 

When animation mode is set to '0' the AnimationManager returns the values of the drivers to the way they were before deletion occured. 


# Critical
The blender_api modification made it necessary that we specifiy mapping parameters between the two types of blendshapes(the one in sophia's rig and the blendshape of faceshift) that we used a specific model of the 
faceshift model to create a more suitable mapping. This made the total shapekeys being sent form the faceshift interface about 57. This are described in the faceshift profile found the profiles folder. So be sure to 
have that otherwise it would give out an error saying that a certain shapekey is missing. And a mapping file is placed in the robots_config/shapekey_pairing.json

# TODOs
* Use the Parameters to set the animation mode rather than topics. 
* Make a UI to easily set new relations between blendshapes between the two interfaces.
* Handle the errors if the faceshift isn't providing the appropriate mappings between the two parts.
* Handle ways to implement if the faceshift and ROS aren't communicating using the IP and Port Settings specified. 
* Check validity of the shapekey drivers after being deleted. 


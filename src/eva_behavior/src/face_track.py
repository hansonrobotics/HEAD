#
# face_track.py -  Registery and tracking of faces
# Copyright (C) 2014,2015  Hanson Robotics
# Copyright (C) 2015 Linas Vepstas
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

import time

from owyl import blackboard
import rospy
from pi_face_tracker.msg import FaceEvent, Faces
from blender_api_msgs.msg import Target
import tf
import random
import math
import logging

from std_msgs.msg import Int32

logger = logging.getLogger('hr.eva_behavior.face_track')

# A Face. Currently consists only of an ID number, a 3D location,
# and the time it was last seen.  Should be extended to include
# the size of the face, possibly the location of the eyes, and,
# if possible, the name of the human attached to it ...
class Face:
    def __init__(self, fid, point):
        self.faceid = fid
        self.x = point.x
        self.y = point.y
        self.z = point.z
        self.t = time.time()

# A registery (in-memory database) of all human faces that are currently
# visible, or have been recently seen.  Implements various basic look-at
# actions, including:
# *) turning to face a given face
# *) tracking a face with the eyes
# *) glancing a currrently-visible face, or a face that was recently
#    seen.
#
# Provides the new-face, lost-face data to general-behavior, by putting
# the face data into the owyl blackboard.
class FaceTrack:

    def __init__(self, owyl_bboard):

        logger.info("Starting Face Tracker")
        self.blackboard = owyl_bboard

        # List of currently visible faces or blobs
        self.visible_faces_blobs = []
        # List of locations of currently visible faces
        self.face_locations = {}

        # List of no longer visible faces, but seen recently.
        self.recent_locations = {}
        # How long (in seconds) to keep around a recently seen, but now
        # lost face. tf does the tracking for us.
        self.RECENT_INTERVAL = 20

        # Current look-at-target
        self.look_at = 0
        self.gaze_at = 0
        self.glance_at = 0
        self.first_glance = -1
        self.glance_howlong = -1

        # How often we update the look-at target.
        self.LOOKAT_INTERVAL = 0.1
        self.last_lookat = 0

        # Last time that the list of active faces was vacuumed out.
        self.last_vacuum = 0
        self.VACUUM_INTERVAL = 1

        # Subscribed pi_vision topics and events
        self.TOPIC_FACE_EVENT = "/camera/face_event"
        self.EVENT_NEW_FACE = "new_face"
        self.EVENT_LOST_FACE = "lost_face"
        self.EVENT_NEW_BLOB = "new_blob"
        self.EVENT_LOST_BLOB = "lost_blob"
        # Overrides current face beeiing tracked by WebUI
        self.EVENT_TRACK_FACE = "track_face"
        self.EVENT_START_TALKING = "started_talking"
        self.EVENT_STOP_TALKING = "stopped_talking"
        self.EVENT_RECOGNIZE_FACE = "recognized_"

        # Publishes the current tracked face
        self.TOPIC_LOOK_AT_FACE = "look_at_face"

        self.TOPIC_FACE_LOCATIONS = "/camera/face_locations"
        self.TOPIC_RS_FACE_LOCATIONS = "/rs_camera/face_locations"

        # Published blender_api topics
        self.TOPIC_FACE_TARGET = "/blender_api/set_face_target"
        self.TOPIC_GAZE_TARGET = "/blender_api/set_gaze_target"

        # Face appearance/disappearance from pi_vision
        rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

        # Face location information from pi_vision
        rospy.Subscriber(self.TOPIC_FACE_LOCATIONS, Faces, self.face_loc_cb)
        rospy.Subscriber(self.TOPIC_RS_FACE_LOCATIONS, Faces, self.rs_face_loc_cb)

        # Where to look
        self.look_pub = rospy.Publisher(self.TOPIC_FACE_TARGET,
            Target, queue_size=10)

        self.gaze_pub = rospy.Publisher(self.TOPIC_GAZE_TARGET,
            Target, queue_size=10)

        self.look_at_face_pub = rospy.Publisher(self.TOPIC_LOOK_AT_FACE,
            Int32, queue_size=10)


        # Frame in which coordinates will be returned from transformation
        self.LOCATION_FRAME = "blender"
        # Transform Listener.Allows history of RECENT_INTERVAL
        self.tf_listener = tf.TransformListener(False, rospy.Duration(self.RECENT_INTERVAL))

    # ---------------------------------------------------------------
    # Public API. Use these to get things done.

    # Turn only the eyes towards the given target face; track that face.
    def gaze_at_face(self, faceid):
        logger.info("gaze at: " + str(faceid))

        # Look at neutral position, 1 meter in front
        if 0 == faceid :
            trg = Target()
            trg.x = 1.0
            trg.y = 0.0
            trg.z = 0.0
            self.gaze_pub.publish(trg)

        self.last_lookat = 0
        if faceid not in self.visible_faces_blobs :
            self.gaze_at = 0
            return

        self.gaze_at = faceid

    # Turn entire head to look at the given target face. The head-turn is
    # performed only once per call; after that, the eyes will then
    # automatically track that face, but the head will not.  Call again,
    # to make the head move again.
    #
    def look_at_face(self, faceid):
        logger.info("look at: " + str(faceid))

        # Look at neutral position, 1 meter in front
        if 0 == faceid :
            trg = Target()
            trg.x = 1.0
            trg.y = 0.0
            trg.z = 0.0
            self.look_pub.publish(trg)

        self.last_lookat = 0
        if faceid not in self.visible_faces_blobs :
            self.look_at = 0
            return

        self.look_at = faceid

    def glance_at_face(self, faceid, howlong):
        logger.info("glance at: " + str(faceid) + " for " + str(howlong) + " seconds")
        self.glance_at = faceid
        self.glance_howlong = howlong
        self.first_glance = -1

    def study_face(self, faceid, howlong):
        logger.info("study: " + str(faceid) + " for " + str(howlong) + " seconds")
        self.glance_at = faceid
        self.glance_howlong = howlong
        self.first_glance = -1

    # ---------------------------------------------------------------
    # Private functions, not for use outside of this class.
    # Add a face to the Owyl blackboard.
    def add_face_to_bb(self, faceid):
        # We already know about it.
        if faceid in self.blackboard["background_face_targets"]:
            return

        # Update the blackboard.
        self.blackboard["is_interruption"] = True
        self.blackboard["new_face"] = faceid
        self.blackboard["background_face_targets"].append(faceid)

    # Add a talking face to Owyl blackboard.
    def add_talking_face_to_bb(self, faceid):
        if faceid in self.blackboard["background_talking_faces"]:
            return

        self.blackboard["background_talking_faces"].append(faceid)
        self.blackboard["is_interruption"] = True

    # Remove a face from the Owyl blackboard.
    def remove_face_from_bb(self, fid):
        if fid not in self.blackboard["background_face_targets"]:
            return

        # Update the blackboard.
        self.blackboard["lost_face"] = fid
        self.blackboard["background_face_targets"].remove(fid)
        # If it is a recognized face, remove it as well
        if fid in self.blackboard["background_x_recognized_face_targets"]:
            self.blackboard["background_x_recognized_face_targets"].remove(fid)
        # If it is a talking face, remove it as well
        if fid in self.blackboard["background_talking_faces"]:
            self.blackboard["background_talking_faces"].remove(fid)
        self.blackboard["is_interruption"] = True
        # If the robot lost the new face during the initial
        # interaction, reset new_face variable
        if self.blackboard["new_face"] == fid :
            self.blackboard["new_face"] = ""

    # Remove a talking face from the Owyl blackboard.
    def remove_talking_face_from_bb(self, faceid):
        if faceid not in self.blackboard["background_talking_faces"]:
            return

        self.blackboard["background_talking_faces"].remove(faceid)

    # Add a blob to the Owyl blackboard.
    def add_blob_to_bb(self, blob_id):
        if blob_id in self.blackboard["background_blob_targets"]:
            return

        if blob_id not in self.blackboard["background_face_targets"]:
            self.blackboard["background_face_targets"].append(blob_id)

        self.blackboard["background_blob_targets"].append(blob_id)

    # Remove a blob from the Owyl blackboard.
    def remove_blob_from_bb(self, blob_id):
        if blob_id not in self.blackboard["background_blob_targets"]:
            return

        self.blackboard["background_blob_targets"].remove(blob_id)

    # Add a recognized face to the Owyl blackboard.
    def add_recognized_face_to_bb(self, faceid, name):
        if faceid in self.blackboard["background_recognized_face_targets"]:
            return
        if faceid in self.blackboard["background_x_recognized_face_targets"]:
            return

        self.blackboard["background_recognized_face_targets"].append(faceid)
        self.blackboard["background_x_recognized_face_targets"].append(faceid)
        self.blackboard["recog_face"] = faceid
        self.blackboard["recog_face_name"] = name
        self.blackboard["is_interruption"] = True

    # Start tracking a face
    def add_face(self, faceid):
        if faceid in self.visible_faces_blobs:
            return

        self.visible_faces_blobs.append(faceid)

        logger.info("New face added to visible faces/blobs: " +
            str(self.visible_faces_blobs))

        self.add_face_to_bb(faceid)

    # Start tracking a blob
    def add_blob(self, blob_id):
        if blob_id in self.visible_faces_blobs:
            return

        self.visible_faces_blobs.append(blob_id)

        logger.info("New blob added to visible faces/blobs: " + str(self.visible_faces_blobs))

        self.add_blob_to_bb(blob_id)

    # Start tracking a talking face
    def add_talking_face(self, faceid):
        self.add_talking_face_to_bb(faceid)
        logger.info("New talking face added: " + str(faceid))

    # Start tracking a recognized face
    def add_recognized_face(self, faceid, name):
        self.add_recognized_face_to_bb(faceid, name)
        logger.info("New recognized face added: " + name + " \(" + str(faceid) + "\)")

    # Stop tracking a face
    def remove_face(self, faceid):
        self.remove_face_from_bb(faceid)
        if faceid in self.visible_faces_blobs:
            self.visible_faces_blobs.remove(faceid)

        logger.info("Lost face; visible faces/blobs now: " + str(self.visible_faces_blobs))

    # Stop tracking a talking face
    def remove_talking_face(self, faceid):
        self.remove_talking_face_from_bb(faceid)
        logger.info("Removed a talking face: " + str(faceid))

    # Stop tracking a blob
    def remove_blob(self, blob_id):
        self.remove_blob_from_bb(blob_id)
        if blob_id in self.visible_faces_blobs:
            self.visible_faces_blobs.remove(blob_id)

        logger.info("Lost blob; visible faces/blobs now: " + str(self.visible_faces_blobs))

    # ----------------------------------------------------------
    # Main look-at action driver.  Should be called at least a few times
    # per second.  This publishes all of the eye-related actions that the
    # blender api robot head should be performing.
    #
    # This performs multiple actions:
    # 1) updates the list of currently visible faces
    # 2) updates the list of recently seen (but now lost) faces
    # 3) If we should be looking at one of these faces, then look
    #    at it, now.
    def do_look_at_actions(self) :
        now = time.time()

        # Should we be glancing elsewhere? If so, then do it, and
        # do it actively, i.e. track that face intently.
        if 0 < self.glance_at:
            if self.first_glance < 0:
                self.first_glance = now
            if (now - self.first_glance < self.glance_howlong):
                face = None

                # Find latest position known
                try:
                    current_trg = self.face_target(self.blackboard["current_face_target"])
                    gaze_trg = self.face_target(self.glance_at)
                    self.glance_or_look_at(current_trg, gaze_trg)
                except:
                    logger.error("no face to glance at!")
                    self.glance_at = 0
                    self.first_glance = -1
            else :
                # We are done with the glance. Resume normal operations.
                self.glance_at = 0
                self.first_glance = -1

        # Publish a new lookat target to the blender API
        elif (now - self.last_lookat > self.LOOKAT_INTERVAL):
            self.last_lookat = now

            # Update the eye position, if need be. Skip, if there
            # is also a pending look-at to perform.

            if 0 < self.gaze_at and self.look_at <= 0:
                # logger.info("Gaze at id " + str(self.gaze_at))
                try:
                    if not self.gaze_at in self.visible_faces_blobs:
                        raise Exception("Face/Blob not visible")
                    current_trg = self.face_target(self.blackboard["current_face_target"])
                    gaze_trg = self.face_target(self.gaze_at)
                    self.glance_or_look_at(current_trg, gaze_trg)
                except tf.LookupException as lex:
                    logger.warn("TF has forgotten about face id:" +
                        str(self.look_at))
                    self.remove_face(self.look_at)
                    self.look_at_face(0)
                    return
                except Exception as ex:
                    logger.error("no gaze-at target")
                    self.gaze_at_face(0)
                    return

            if 0 < self.look_at:
                logger.info("Look at id: " + str(self.look_at))
                try:
                    if not self.look_at in self.visible_faces_blobs:
                        raise Exception("Face/Blob not visible")
                    trg = self.face_target(self.look_at)
                    self.look_pub.publish(trg)
                except tf.LookupException as lex:
                    logger.warn("TF has forgotten about face id: " +
                        str(self.look_at))
                    self.remove_face(self.look_at)
                    self.look_at_face(0)
                    return
                except Exception as ex:
                    logger.error("no look-at target")
                    self.look_at_face(0)
                    return

                # Now that we've turned to face the target, don't do it
                # again; instead, just track with the eyes.
                self.gaze_at = self.look_at
                self.look_at = -1

    # If the distance between the current face target and the glace_at target > max_glance_distance
    # Look at that face instead (so that the neck will also move instead of the eyes only)
    def glance_or_look_at(self, current_trg, gaze_trg):
        z = (current_trg.z - gaze_trg.z)
        # Avoid division by zero
        if z == 0:
            z = 1
        gaze_distance = math.sqrt(math.pow((current_trg.x - gaze_trg.x), 2) + \
                        math.pow((current_trg.y - gaze_trg.y), 2)) / z
        if gaze_distance > self.blackboard["max_glance_distance"]:
            logger.info("Reached max_glance_distance, look at the face instead")
            self.look_pub.publish(gaze_trg)
        else:
            # For face study saccade
            if self.blackboard["face_study_nose"]:
                gaze_trg.z += self.blackboard["face_study_z_pitch_nose"]
            elif self.blackboard["face_study_mouth"]:
                gaze_trg.z += self.blackboard["face_study_z_pitch_mouth"]
            elif self.blackboard["face_study_left_ear"]:
                gaze_trg.y += self.blackboard["face_study_y_pitch_left_ear"]
            elif self.blackboard["face_study_right_ear"]:
                gaze_trg.y += self.blackboard["face_study_y_pitch_right_ear"]

            # Publish the gaze_at ROS message
            self.gaze_pub.publish(gaze_trg)

            # Reset face study saccade related flags
            self.blackboard["face_study_nose"] = False
            self.blackboard["face_study_mouth"] = False
            self.blackboard["face_study_left_ear"] = False
            self.blackboard["face_study_right_ear"] = False


    # ----------------------------------------------------------
    # pi_vision ROS callbacks

    # pi_vision ROS callback, called when a new face is detected,
    # or a face is lost.  Note: I don't think this is really needed,
    # the face_loc_cb accomplishes the same thing. So maybe should
    # remove this someday.
    def face_event_cb(self, data):
        if not self.blackboard["behavior_tree_on"]:
            return

        if data.face_event.startswith(self.EVENT_RECOGNIZE_FACE):
            # Extract the name from the event name
            idx = len(self.EVENT_RECOGNIZE_FACE)
            name = data.face_event[idx:]
            self.add_recognized_face(data.face_id, name)

        elif data.face_event == self.EVENT_NEW_FACE:
            self.add_face(data.face_id)

        elif data.face_event == self.EVENT_LOST_FACE:
            self.remove_face(data.face_id)

        elif data.face_event == self.EVENT_TRACK_FACE:
            self.blackboard['new_look_at_face'] = data.face_id
            self.blackboard['is_interruption'] = True

        elif data.face_event == self.EVENT_START_TALKING:
            self.add_talking_face(data.face_id)

        elif data.face_event == self.EVENT_STOP_TALKING:
            self.remove_talking_face(data.face_id)

        elif data.face_event == self.EVENT_NEW_BLOB:
            self.add_blob(data.face_id)

        elif data.face_event == self.EVENT_LOST_BLOB:
            self.remove_blob(data.face_id)

    def rs_face_loc_cb(self,data):
        if not self.blackboard["behavior_tree_on"]:
            return
        current_faces = []
        for face in data.faces:
            fid = face.id
            current_faces.append(fid)
            if fid not in self.blackboard['rs_face_targets']:
                self.blackboard['rs_face_targets'].append(fid)
        for f in self.blackboard['rs_face_targets']:
            if f not in current_faces:
                self.remove_face_from_bb(f)
        self.face_loc_cb(data)

    # pi_vision ROS callback, called when pi_vision has new face
    # location data for us. Because this happens frequently (10x/second)
    # we also use this as the main update loop, and drive all look-at
    # actions from here.
    def face_loc_cb(self, data):
        if not self.blackboard["behavior_tree_on"]:
            return

        for face in data.faces:
            fid = face.id
            loc = face.point
            # Sanity check.  Sometimes pi_vision sends us faces with
            # location (0,0,0). Discard these.
            if loc.x < 0.05:
                continue
            self.add_face(fid)

        # Now perform all the various looking-at actions
        self.do_look_at_actions()

    # Queries tf_listener to get latest available position
    # Throws TF exceptions if transform canot be returned
    def face_target(self, faceid):
        (trans, rot) = self.tf_listener.lookupTransform( \
            self.LOCATION_FRAME, 'Face' + str(faceid), rospy.Time(0))
        t = Target()
        t.x = trans[0]
        t.y = trans[1]
        t.z = trans[2] + self.blackboard["z_pitch_eyes"]
        return t


    # Picks random face from current visible faces
    # Prioritizes the real faces over virtual faces in attention regions
    @staticmethod
    def random_face_target(faces, exclude = 0):
        if len(faces) < 1:
            return 0
        # Faces with smaller (less than <1,000,000 ids are prioritized
        small_ids = [f for f in faces if (f < 1000000) and (f != exclude)]
        if len(small_ids) < 1:
            # return random.choice(small_ids)
            return random.choice(faces)
        else:
            return random.choice(small_ids)

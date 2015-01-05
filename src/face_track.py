#
# face_track.py -  Registery and tracking of faces
# Copyright (C) 2014  Hanson Robotics
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

from owyl import blackboard
import rospy
from pi_face_tracker.msg import FaceEvent

# A registery of all the faces currently visible.
# Copies face data to owyl blackboard.
class FaceTrack:

	def __init__(self, owyl_bboard):

		print("Starting Face Tracker")
		self.blackboard = owyl_bboard
		self.visible_faces = []

		# pi_vision topics and events
		self.TOPIC_FACE_EVENT = "face_event"
		self.EVENT_NEW_FACE = "new_face"
		self.EVENT_LOST_FACE = "lost_face"

		# Face appearance/disappearance from pi_vision
		rospy.Subscriber(self.TOPIC_FACE_EVENT, FaceEvent, self.face_event_cb)

		# Face location information from pi_vision

	def face_event_cb(self, data):
		self.blackboard["is_interruption"] = True
		if data.face_event == self.EVENT_NEW_FACE:
			print "<< Interruption >> New Face Detected: " + str(data.face_id)
			self.blackboard["new_face"] = str(data.face_id)
			self.blackboard["background_face_targets"].append(self.blackboard["new_face"])
		elif data.face_event == self.EVENT_LOST_FACE:
			print "<< Interruption >> Lost Face Detected: " + str(data.face_id)
			if data.face_id in self.blackboard["background_face_targets"]:
				self.blackboard["lost_face"] = str(data.face_id)
				self.blackboard["background_face_targets"].remove(self.blackboard["lost_face"])
				# If the robot lost the new face during the initial
				# interaction, reset new_face variable
				if self.blackboard["new_face"] == self.blackboard["lost_face"]:
					self.blackboard["new_face"] = ""

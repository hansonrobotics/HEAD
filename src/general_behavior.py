#
# general_behavior.py -  The primary Owyl behavior tree
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

# system imports
import copy
import os
import random
import time

# tool imports
import owyl
from owyl import blackboard
import rospy
import roslib
import ConfigParser

# message imports
from std_msgs.msg import String
from blender_api_msgs.msg import AvailableEmotionStates, AvailableGestures
from blender_api_msgs.msg import EmotionState
from blender_api_msgs.msg import SetGesture

# local stuff.
from face_track import FaceTrack

# Basic holder for emotion-expression properties and probabilities
class Emotion:
	def __init__(self, name) :
		self.name = name
		self.probability = 0.0
		self.min_intensity = 0.0
		self.max_intensity = 1.0
		self.min_duration = 5.0
		self.max_duration = 15.0

class Tree():
	# ---------
	# Config File utilities
	def unpack_config_emotions(self, config, emo_class) :

		def get_values(from_config, num_values):
			rtn_values = [float(z.strip()) for z in from_config.split(",")]
			if len(rtn_values) != num_values:
				raise Exception("List lengths don't match!")
			return rtn_values

		names = [x.strip() for x in config.get("emotion", emo_class).split(",")]
		numb = len(names)

		probs = get_values(config.get("emotion", \
			emo_class + "_probabilities"), numb)
		mins = get_values(config.get("emotion", \
			emo_class + "_intensity_min"), numb)
		maxs = get_values(config.get("emotion", \
			emo_class + "_intensity_max"), numb)

		dins = get_values(config.get("emotion", \
			emo_class + "_duration_min"), numb)
		daxs = get_values(config.get("emotion", \
			emo_class + "_duration_max"), numb)

		emos = []
		for (n,p,mi,mx,di,dx) in zip (names, probs, mins, maxs, dins, daxs):
			emo = Emotion(n)
			emo.probability = p
			emo.min_intensity = mi
			emo.max_intensity = mx
			emo.min_duration = di
			emo.max_duration = dx
			emos.append(emo)

		self.blackboard[emo_class] = emos

	def __init__(self):

		self.blackboard = blackboard.Blackboard("rig expressions")

		config = ConfigParser.ConfigParser()
		config.readfp(open(os.path.join(os.path.dirname(__file__), "../behavior.cfg")))
		self.blackboard["sadness_happiness"] = config.getfloat("emotion", "sadness_happiness")
		self.blackboard["irritation_amusement"] = config.getfloat("emotion", "irritation_amusement")
		self.blackboard["confusion_comprehension"] = config.getfloat("emotion", "confusion_comprehension")
		self.blackboard["boredom_engagement"] = config.getfloat("emotion", "boredom_engagement")
		self.blackboard["recoil_surprise"] = config.getfloat("emotion", "recoil_surprise")
		self.blackboard["emotions"] = [x.strip() for x in config.get("emotion", "basic_emotions").split(",")]
		self.blackboard["current_emotion"] = config.get("emotion", "default_emotion")
		self.blackboard["current_emotion_intensity"] = config.getfloat("emotion", "default_emotion_intensity")

		self.unpack_config_emotions(config, "frustrated_emotions")

		self.unpack_config_emotions(config, "positive_emotions")
		self.unpack_config_emotions(config, "non_positive_emotion")

		self.unpack_config_emotions(config, "bored_emotions")
		self.unpack_config_emotions(config, "non_bored_emotion")

		self.unpack_config_emotions(config, "sleep_emotions")
		self.unpack_config_emotions(config, "non_sleep_emotion")

		self.unpack_config_emotions(config, "wake_up_emotions")

		self.blackboard["min_duration_for_interaction"] = config.getfloat("interaction", "duration_min")
		self.blackboard["max_duration_for_interaction"] = config.getfloat("interaction", "duration_max")
		self.blackboard["time_to_change_face_target_min"] = config.getfloat("interaction", "time_to_change_face_target_min")
		self.blackboard["time_to_change_face_target_max"] = config.getfloat("interaction", "time_to_change_face_target_max")
		self.blackboard["glance_probability"] = config.getfloat("interaction", "glance_probability")
		self.blackboard["glance_probability_for_new_faces"] = config.getfloat("interaction", "glance_probability_for_new_faces")
		self.blackboard["glance_probability_for_lost_faces"] = config.getfloat("interaction", "glance_probability_for_lost_faces")
		self.blackboard["sleep_probability"] = config.getfloat("boredom", "sleep_probability")
		self.blackboard["sleep_duration_min"] = config.getfloat("boredom", "sleep_duration_min")
		self.blackboard["sleep_duration_max"] = config.getfloat("boredom", "sleep_duration_max")
		self.blackboard["search_for_attention_duration_min"] = config.getfloat("boredom", "search_for_attention_duration_min")
		self.blackboard["search_for_attention_duration_max"] = config.getfloat("boredom", "search_for_attention_duration_max")
		self.blackboard["wake_up_probability"] = config.getfloat("boredom", "wake_up_probability")
		self.blackboard["time_to_wake_up"] = config.getfloat("boredom", "time_to_wake_up")

		##### Other System Variables #####
		self.blackboard["show_expression_since"] = time.time()

		# ID's of faces newly seen, or lost. Integer ID.
		self.blackboard["new_face"] = 0
		self.blackboard["lost_face"] = 0
		# IDs of faces in the scene, updated once per cycle
		self.blackboard["face_targets"] = []
		# IDs of faces in the scene, updated immediately
		self.blackboard["background_face_targets"] = []
		self.blackboard["current_glance_target"] = ""
		self.blackboard["current_face_target"] = ""
		self.blackboard["interact_with_face_target_since"] = 0.0
		self.blackboard["sleep_since"] = 0.0
		self.blackboard["is_interruption"] = False
		self.blackboard["is_sleeping"] = False
		self.blackboard["blender_mode"] = ""
		self.blackboard["is_scripted_performance_system_on"] = True
		self.blackboard["random"] = 0.0

		##### ROS Connections #####
		self.facetrack = FaceTrack(self.blackboard)

		rospy.Subscriber("behavior_switch", String, self.behavior_switch_callback)
		rospy.Subscriber("/blender_api/available_emotion_states",
			AvailableEmotionStates, self.get_emotion_states_cb)

		rospy.Subscriber("/blender_api/available_gestures",
			AvailableGestures, self.get_gestures_cb)

		# cmd_blendermode needs to go away eventually...
		self.tracking_mode_pub = rospy.Publisher("/cmd_blendermode", String, queue_size=1, latch=True)
		self.emotion_pub = rospy.Publisher("/blender_api/set_emotion_state", EmotionState, queue_size=1)
		self.gesture_pub = rospy.Publisher("/blender_api/set_gesture", SetGesture, queue_size=1)
		self.tree = self.build_tree()
		time.sleep(0.1)

		while True:
			self.tree.next()


	# Pick a random expression out of the class of expressions,
	# and display it. Return the display emotion, or None if none
	# were picked.
	def pick_random_expression(self, emo_class_name):
		random_number = random.random()
		tot = 0
		emo = None
		emos = self.blackboard[emo_class_name]
		for emo in emos:
			tot += emo.probability
			if random_number <= tot:
				break

		if emo:
			intensity = random.uniform(emo.min_intensity, emo.max_intensity)
			duration = random.uniform(emo.min_duration, emo.max_duration)
			self.show_emotion(emo.name, intensity, duration)

		return emo


	# Pick the name of a random emotion, excluding those from
	# the exclude list
	def pick_random_emotion_name(self, exclude) :
		ixnay = [ex.name for ex in exclude]
		emos = self.blackboard["emotions"]
		emo_name = random.choice([other for other in emos if other not in ixnay])
		return emo_name

	# Pick a  so-called "instant" or "flash" expression
	def pick_instant(self, emo_class, exclude_class) :
		emo = self.pick_random_expression(exclude_class)
		if emo :
			exclude = self.blackboard[emo_class]
			emo_name = self.pick_random_emotion_name(exclude)
			tense = random.uniform(emo.min_intensity, emo.max_intensity)
			durat = random.uniform(emo.min_duration, emo.max_duration)
			self.show_emotion(emo_name, tense, durat)
			time.sleep(durat) # XXX Sleep is a bad idea, blocks events ...
		return emo_name

	# ------------------------------------------------------------------
	# The various behavior trees

	# Actions that are taken when a face becomes visible.
	def someone_arrived(self) :
		tree = owyl.sequence(
			self.is_someone_arrived(),
			self.set_emotion(variable="boredom_engagement", value=0.5),
			owyl.selector(
				##### There were no people in the scene #####
				owyl.sequence(
					self.were_no_people_in_the_scene(),
					self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.2, max=1.4),
					self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.2, max=1.4),
					self.assign_face_target(variable="current_face_target", value="new_face"),
					self.record_start_time(variable="interact_with_face_target_since"),
					self.interact_with_face_target(id="current_face_target", new_face=True)
				),

				##### Currently interacting with someone #####
				owyl.sequence(
					self.is_interacting_with_someone(),
					self.is_random_smaller_than(val1="newRandom", val2="glance_probability_for_new_faces"),
					self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.05, max=1.1),
					self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.05, max=1.1),
					self.glance_at_new_face()
				),

				##### Does Nothing #####
				owyl.sequence(
					self.print_status(str="----- Ignoring The New Face!"),
					self.does_nothing()
				)
			),
			self.clear_new_face_target()
		)
		return tree

	# Actions that are taken when a face leaves
	def someone_left(self) :
		tree = owyl.sequence(
			self.is_someone_left(),
			owyl.selector(
				##### Was Interacting With That Person #####
				owyl.sequence(
					self.was_interacting_with_that_person(),
					self.update_emotion(variable="confusion_comprehension", lower_limit=0.0, min=0.4, max=0.6),
					self.update_emotion(variable="recoil_surprise", lower_limit=0.0, min=1.8, max=2.2),
					self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.4, max=0.6),
					self.update_emotion(variable="irritation_amusement", lower_limit=0.0, min=0.95, max=1.0),
					self.show_frustrated_expression()
				),

				##### Is Interacting With Someone Else #####
				owyl.sequence(
					self.is_interacting_with_someone(),
					self.is_random_smaller_than(val1="newRandom", val2="glance_probability_for_lost_faces"),
					self.glance_at_lost_face()
				),

				##### Does Nothing #####
				owyl.sequence(
					self.print_status(str="----- Ignoring The Lost Face!"),
					self.does_nothing()
				)
			),
			self.clear_lost_face_target()
		)
		return tree

	# ------------------------------------------------------------------
	# Build the main tree
	def build_tree(self):
		eva_behavior_tree = \
			owyl.repeatAlways(
				owyl.selector(
					owyl.sequence(
						self.is_scripted_performance_system_off(),
						self.sync_variables(),
						########## Main Events ##########
						owyl.selector(
							self.someone_arrived(),
							self.someone_left(),

							##### People Interaction #####
							owyl.sequence(
								self.is_face_target(),
								self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.001, max=1.005),
								self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.005, max=1.01),
								owyl.selector(
									##### Start A New Interaction #####
									owyl.sequence(
										owyl.selector(
											self.is_not_interacting_with_someone(),
											owyl.sequence(
												self.is_more_than_one_face_target(),
												self.is_time_to_change_face_target()
											)
										),
										self.select_a_face_target(),
										self.record_start_time(variable="interact_with_face_target_since"),
										self.interact_with_face_target(id="current_face_target", new_face=False)
									),

									##### Glance At Other Faces & Continue With The Last Interaction #####
									owyl.sequence(
										self.print_status(str="----- Continue The Interaction"),
										owyl.selector(
											owyl.sequence(
												self.is_more_than_one_face_target(),
												self.is_random_smaller_than(val1="newRandom", val2="glance_probability"),
												self.select_a_glance_target(),
												self.glance_at(id="current_glance_target")
											),
											self.does_nothing()
										),
										self.interact_with_face_target(id="current_face_target", new_face=False)
									)
								)
							),

							##### Nothing Interesting Is Happening #####
							owyl.sequence(
								self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=0.8, max=0.9),
								self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.995, max=1.0),
								owyl.selector(
									##### Is Not Sleeping #####
									owyl.sequence(
										self.is_not_sleeping(),
										owyl.selector(
											##### Go To Sleep #####
											owyl.sequence(
												self.is_random_smaller_than(val1="newRandom_plus_boredom", val2="sleep_probability"),
												self.record_start_time(variable="sleep_since"),
												self.print_status(str="----- Go To Sleep!"),
												self.go_to_sleep()
											),

											##### Search For Attention #####
											self.search_for_attention()
										)
									),

									##### Is Sleeping #####
									owyl.selector(
										##### Wake Up #####
										owyl.sequence(
											self.is_random_smaller_than(val1="newRandom", val2="wake_up_probability"),
											self.is_time_to_wake_up(),
											self.wake_up(),
											self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
										),

										##### Continue To Sleep #####
										owyl.sequence(
											self.print_status(str="----- Continue To Sleep!"),
											self.go_to_sleep()
										)
									)
								),

								##### If Interruption && Sleeping -> Wake Up #####
								owyl.sequence(
									self.is_interruption(),
									self.is_sleeping(),
									self.wake_up(),
									self.print_status(str="----- Interruption: Wake Up!"),
									self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
								)
							)
						)
					),

					############### Scripted Performance System ###############
					# This only executes when scripting is turned off.
					owyl.sequence(
						self.idle_spin(),
						self.is_scripted_performance_system_on(),
						self.start_scripted_performance_system()
					)
				)
			)
		return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

	# Print a single status message
	@owyl.taskmethod
	def print_status(self, **kwargs):
		print kwargs["str"]
		yield True

	# Print emotional state
	@owyl.taskmethod
	def sync_variables(self, **kwargs):
		self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
		print "\n========== Emotion Space =========="
		print "sadness_happiness: " + str(self.blackboard["sadness_happiness"])[:5]
		print "irritation_amusement: " + str(self.blackboard["irritation_amusement"])[:5]
		print "confusion_comprehension: " + str(self.blackboard["confusion_comprehension"])[:5]
		print "boredom_engagement: " + str(self.blackboard["boredom_engagement"])[:5]
		print "recoil_surprise: " + str(self.blackboard["recoil_surprise"])[:5]
		print "Current Emotion: " + self.blackboard["current_emotion"] + " (" + str(self.blackboard["current_emotion_intensity"])[:5] + ")"
		yield True

	@owyl.taskmethod
	def does_nothing(self, **kwargs):
		yield True

	@owyl.taskmethod
	def set_emotion(self, **kwargs):
		self.blackboard[kwargs["variable"]] = kwargs["value"]
		yield True

	@owyl.taskmethod
	def update_emotion(self, **kwargs):
		if kwargs["lower_limit"] > 0.0:
			self.blackboard[kwargs["variable"]] = kwargs["lower_limit"]
		self.blackboard[kwargs["variable"]] *= random.uniform(kwargs["min"], kwargs["max"])
		if self.blackboard[kwargs["variable"]] > 1.0:
			self.blackboard[kwargs["variable"]] = 1.0
		elif self.blackboard[kwargs["variable"]] <= 0.0:
			self.blackboard[kwargs["variable"]] = 0.01
		yield True

	@owyl.taskmethod
	def is_random_smaller_than(self, **kwargs):
		if kwargs["val1"] == "newRandom":
			self.blackboard["random"] = random.random()
		elif kwargs["val1"] == "newRandom_plus_boredom":
			self.blackboard["random"] = random.random() + self.blackboard["boredom_engagement"]
		if self.blackboard["random"] < self.blackboard[kwargs["val2"]]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_someone_arrived(self, **kwargs):
		self.blackboard["is_interruption"] = False
		if self.blackboard["new_face"] > 0:
			print "----- Someone Arrived!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_someone_left(self, **kwargs):
		self.blackboard["is_interruption"] = False
		if self.blackboard["lost_face"] > 0:
			print "----- Someone Left!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_interacting_with_someone(self, **kwargs):
		if self.blackboard["current_face_target"]:
			"----- Is Interacting With Someone!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_not_interacting_with_someone(self, **kwargs):
		if not self.blackboard["current_face_target"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def were_no_people_in_the_scene(self, **kwargs):
		if len(self.blackboard["face_targets"]) == 1:
			print "----- Were No People In The Scene!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def was_interacting_with_that_person(self, **kwargs):
		if self.blackboard["current_face_target"] == self.blackboard["lost_face"]:
			self.blackboard["current_face_target"] = 0
			print "----- Was Interacting With That Person!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_face_target(self, **kwargs):
		if len(self.blackboard["face_targets"]) > 0:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_more_than_one_face_target(self, **kwargs):
		if len(self.blackboard["face_targets"]) > 1:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_time_to_change_face_target(self, **kwargs):
		if self.blackboard["interact_with_face_target_since"] > 0 and (time.time() - self.blackboard["interact_with_face_target_since"]) >= random.uniform(self.blackboard["time_to_change_face_target_min"], self.blackboard["time_to_change_face_target_max"]):
			print "----- Time To Start A New Interaction!"
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_time_to_wake_up(self, **kwargs):
		if self.blackboard["sleep_since"] > 0 and (time.time() - self.blackboard["sleep_since"]) >= self.blackboard["time_to_wake_up"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_sleeping(self, **kwargs):
		if self.blackboard["is_sleeping"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_not_sleeping(self, **kwargs):
		if not self.blackboard["is_sleeping"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_interruption(self, **kwargs):
		if self.blackboard["is_interruption"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_scripted_performance_system_on(self, **kwargs):
		if self.blackboard["is_scripted_performance_system_on"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def is_scripted_performance_system_off(self, **kwargs):
		if not self.blackboard["is_scripted_performance_system_on"]:
			yield True
		else:
			yield False

	@owyl.taskmethod
	def assign_face_target(self, **kwargs):
		self.blackboard[kwargs["variable"]] = self.blackboard[kwargs["value"]]
		yield True

	@owyl.taskmethod
	def select_a_face_target(self, **kwargs):
		self.blackboard["current_face_target"] = random.choice(self.blackboard["face_targets"])
		yield True

	@owyl.taskmethod
	def select_a_glance_target(self, **kwargs):
		target = random.choice(self.blackboard["face_targets"])
		while target == self.blackboard["current_face_target"]:
			target = random.choice(self.blackboard["face_targets"])
		self.blackboard["current_glance_target"] = target
		yield True

	@owyl.taskmethod
	def record_start_time(self, **kwargs):
		self.blackboard[kwargs["variable"]] = time.time()
		yield True

	@owyl.taskmethod
	def interact_with_face_target(self, **kwargs):
		if self.blackboard["blender_mode"] != "TrackDev":
			self.tracking_mode_pub.publish("TrackDev")
			self.blackboard["blender_mode"] = "TrackDev"
			time.sleep(0.1)
		face_id = self.blackboard[kwargs["id"]]
		duration = random.uniform(self.blackboard["min_duration_for_interaction"], self.blackboard["max_duration_for_interaction"])
		interval = 0.01
		self.facetrack.look_at_face(face_id)

		print "----- Interacting w/Face(id:" + str(face_id) + ") for " + str(duration)[:5] + " seconds"
		if time.time() - self.blackboard["show_expression_since"] >= 2.0 or kwargs["new_face"]:
			##### Show A Random Instant Expression #####
			self.pick_instant("positive_emotions", "non_positive_emotion")

			##### Show A Positive Expression #####
			self.pick_random_expression("positive_emotions")

		while duration > 0:
			time.sleep(interval)
			duration -= interval
			if self.blackboard["is_interruption"]:
				break
		yield True

	@owyl.taskmethod
	def glance_at(self, **kwargs):
		face_id = self.blackboard[kwargs["id"]]
		print "----- Glancing at face:" + str(face_id)
		glance_seconds = 1
		self.facetrack.glance_at_face(face_id, glance_seconds)
		yield True

	@owyl.taskmethod
	def glance_at_new_face(self, **kwargs):
		face_id = self.blackboard["new_face"]
		print "----- Glancing at new face:" + str(face_id)
		glance_seconds = 1
		self.facetrack.glance_at_face(face_id, glance_seconds)
		yield True

	@owyl.taskmethod
	def glance_at_lost_face(self, **kwargs):
		print "----- Glancing at lost face:" + str(self.blackboard["lost_face"])
		face_id = self.blackboard["lost_face"]
		self.facetrack.glance_at_face(face_id, 1)
		yield True

	@owyl.taskmethod
	def show_expression(self, **kwargs):
		self.show_emotion(kwargs["expression"], random.uniform(kwargs["min_intensity"], kwargs["max_intensity"]), 15)
		yield True

	@owyl.taskmethod
	def show_frustrated_expression(self, **kwargs):
		self.pick_random_expression("frustrated_emotions")
		yield True

	# Accept an expression name, intentisty and duration, and publish it
	# as a ros message.
	def show_emotion(self, expression, intensity, duration):

		# Update the blackboard
		self.blackboard["current_emotion"] = expression
		self.blackboard["current_emotion_intensity"] = intensity

		# Create the message
		exp = EmotionState()
		exp.name = self.blackboard["current_emotion"]
		exp.magnitude = self.blackboard["current_emotion_intensity"]
		intsecs = int(duration)
		exp.duration.secs = intsecs
		exp.duration.nsecs = 1000000000 * (duration - intsecs)
		self.emotion_pub.publish(exp)

		print "----- Show expression: " + expression + " (" + str(intensity)[:5] + ")"
		self.blackboard["show_expression_since"] = time.time()

	@owyl.taskmethod
	def search_for_attention(self, **kwargs):
		print "----- Search For Attention!"
		duration = random.uniform(self.blackboard["search_for_attention_duration_min"], self.blackboard["search_for_attention_duration_max"])
		if self.blackboard["blender_mode"] != "LookAround":
			self.tracking_mode_pub.publish("LookAround")
			self.blackboard["blender_mode"] = "LookAround"

		if time.time() - self.blackboard["show_expression_since"] >= 5.0:
			##### Show A Random Instant Expression #####
			self.pick_instant("bored_emotions", "non_bored_emotion")

			##### Show A Bored Expression #####
			self.pick_random_expression("bored_emotions")

		interval = 0.01
		while duration > 0:
			time.sleep(interval)
			duration -= interval
			if self.blackboard["is_interruption"]:
				break
		yield True

	@owyl.taskmethod
	def go_to_sleep(self, **kwargs):
		self.blackboard["is_sleeping"] = True
		##### Show A Sleep Expression #####
		self.pick_random_emotion_name("sleep_emotions")

		interval = 0.01
		duration = 15
		#TODO: Topic for sleep
		while duration > 0:
			time.sleep(interval)
			duration -= interval
			if self.blackboard["is_interruption"]:
				break
		yield True

	@owyl.taskmethod
	def wake_up(self, **kwargs):
		print "----- Wake Up!"
		self.blackboard["is_sleeping"] = False
		self.blackboard["sleep_since"] = 0.0
		##### Show A Wake Up Expression #####
		self.pick_random_expression("wake_up_emotions")

		#TODO: Topic for waking up
		yield True

	@owyl.taskmethod
	def clear_new_face_target(self, **kwargs):
		if not self.blackboard["is_interruption"]:
			print "----- Cleared New Face: " + str(self.blackboard["new_face"])
			self.blackboard["new_face"] = 0
		yield True

	@owyl.taskmethod
	def clear_lost_face_target(self, **kwargs):
		print "----- Cleared Lost Face: " + str(self.blackboard["lost_face"])
		self.blackboard["lost_face"] = 0
		yield True

	# XXX old-style API -- should be removed.
	@owyl.taskmethod
	def start_scripted_performance_system(self, **kwargs):
		if self.blackboard["blender_mode"] != "Dummy":
			# No need to set Dummy mode
			#self.tracking_mode_pub.publish("Dummy")
			self.blackboard["blender_mode"] = "Dummy"
		yield True


	# This avoids burning CPU time when teh behavior system is off.
	# Mostly it sleeps, and peridoically checks for interrpt messages.
	@owyl.taskmethod
	def idle_spin(self, **kwargs):
		if self.blackboard["is_scripted_performance_system_on"]:
			yield True

		# Sleep for 1 second.
		time.sleep(1)
		yield True


	# Return the subset of 'core' strings that are in 'avail' strings.
	# Note that 'avail' strings might contain longer names,
	# e.g. "happy-3", whereas core just contains "happy". We want to
	# return "happy-3" in that case, as well as happy-2 and happy-1
	# if they are there.
	def set_intersect(self, emo_class, avail) :
		emos = self.blackboard[emo_class]
		rev = []
		for emo in emos:
			for a in avail:
				if emo.name in a:
					# Copy the emotion, but give it the new name!
					nemo = copy.deepcopy(emo)
					nemo.name = a
					rev.append(nemo)

		# Now, renormalize the probabilities
		tot = 0.0
		for emo  in rev:
			tot += emo.probability
		for emo  in rev:
			emo.probability /= tot

		self.blackboard[emo_class] = rev

	# Get the list of available emotions. Update our master list,
	# and cull the various subclasses appropriately.
	def get_emotion_states_cb(self, msg) :
		print("Available Emotion States:" + str(msg.data))
		# Update the complete list of emtions.
		self.blackboard["emotions"] = msg.data

		# Reconcile the other classes
		self.set_intersect("frustrated_emotions", msg.data)
		self.set_intersect("positive_emotions", msg.data)
		self.set_intersect("bored_emotions", msg.data)
		self.set_intersect("sleep_emotions", msg.data)
		self.set_intersect("wake_up_emotions", msg.data)


	def get_gestures_cb(self, msg) :
		print("Available Gestures:" + str(msg.data))

	# Turn behaviors on and off.
	def behavior_switch_callback(self, data):
		if data.data == "btree_on":
			self.blackboard["is_interruption"] = False
			self.blackboard["is_scripted_performance_system_on"] = False
			print "Behavior Tree is ON!"
		elif data.data == "btree_off":
			self.blackboard["is_interruption"] = True
			self.blackboard["is_scripted_performance_system_on"] = True
			print "Behavior Tree is OFF!"


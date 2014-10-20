#!/usr/bin/python
import owyl
from owyl import blackboard
import rospy
import roslib
import random
import time
from std_msgs.msg import String


class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.blackboard["sadness_happiness"] = 0.0
        self.blackboard["irritation_amusement"] = 0.0
        self.blackboard["confusion_comprehension"] = 0.0
        self.blackboard["boredom_engagement"] = 0.0
        self.blackboard["recoil_surprise"] = 0.0
        self.blackboard["face_targets"] = []  # IDs of faces in the scene
        self.blackboard["background_face_targets"] = []
        self.blackboard["new_face"] = ""
        self.blackboard["lost_face"] = ""
        self.blackboard["current_glance_target"] = ""
        self.blackboard["current_face_target"] = ""
        self.blackboard["interact_with_face_target_since"] = 0.0
        self.blackboard["sleep_since"] = 0.0
        self.blackboard["is_interruption"] = False
        self.blackboard["is_sleeping"] = False
        self.blackboard["is_TrackDev_on"] = False
        self.blackboard["random"] = 0.0
        rospy.Subscriber("tracking_event", String, self.tracking_event_callback)
        self.tracking_mode_pub = rospy.Publisher("tracking_mode", String, queue_size=1)
        self.tracking_action_pub = rospy.Publisher("tracking_action", String, queue_size=1)  # For now, will change the type to "TrackingAction"
        self.face_id_pub = rospy.Publisher("faces/id", String, queue_size=1)
        self.emotion_pub = rospy.Publisher("emotion", String, queue_size=1)  # For now, will change the type to "basic_head_api/MakeFaceExpr"
        self.tree = self.build_tree()
        while True:
            self.tree.next()

    def build_tree(self):
        eva_behavior_tree = \
            owyl.parallel(
                ############### Tree: General Behaviors ###############
                owyl.repeatAlways(
                    owyl.sequence(
                        self.sync_variables(),
                        ########## Main Events ##########
                        owyl.sequence(
                            ##### When Someone Arrived #####
                            owyl.sequence(
                                self.is_someone_arrived(),
                                self.set_emotion(variable="boredom_engagement", value=0.5),
                                owyl.selector(
                                    ##### Were No People In The Scene #####
                                    owyl.sequence(
                                        self.were_no_people_in_the_scene(),
                                        self.update_emotion(variable="sadness_happiness", min=1.2, max=1.4),
                                        self.update_emotion(variable="boredom_engagement", min=1.2, max=1.4),
                                        self.assign_face_target(variable="current_face_target", value="face_targets"),
                                        self.interact_with_face_target(id="current_face_target", min_duration=5, max_duration=10.0),
                                        self.say(utterance="Hello")
                                    ),

                                    ##### Is Interacting With Someone #####
                                    owyl.sequence(
                                        self.is_interacting_with_someone(),
                                        self.is_random_greater_than(val1="newRandom", val2=0.5),
                                        self.update_emotion(variable="sadness_happiness", min=1.05, max=1.1),
                                        self.update_emotion(variable="boredom_engagement", min=1.05, max=1.1),
                                        self.glance_at_new_face(),
                                        self.nod()
                                    ),

                                    ##### Does Nothing #####
                                    owyl.sequence(
                                        self.does_nothing()
                                    )
                                ),
                                self.clear_new_face_target()
                            ),

                            ##### When Someone Left #####
                            owyl.sequence(
                                self.is_someone_left(),
                                owyl.selector(
                                    ##### Was Interacting With That Person #####
                                    owyl.sequence(
                                        self.was_interacting_with_that_person(),
                                        owyl.selector(
                                            ##### Confused #####
                                            owyl.sequence(
                                                self.is_random_smaller_than(val1="newRandom", val2=0.4),
                                                self.update_emotion(variable="confusion_comprehension", min=0.4, max=0.6),
                                                self.show_expression(expression="confused", min_duration=1.0, max_duration=3.0)
                                            ),

                                            ##### Surprised #####
                                            owyl.sequence(
                                                self.is_random_in_between(val="random", greater=0.4, smaller=0.8),
                                                self.update_emotion(variable="confusion_comprehension", min=0.4, max=0.6),
                                                self.show_expression(expression="confused", min_duration=1.0, max_duration=3.0)
                                            ),

                                            ##### Sad #####
                                            owyl.sequence(
                                                self.update_emotion(variable="confusion_comprehension", min=0.4, max=0.6),
                                                self.show_expression(expression="confused", min_duration=1.0, max_duration=3.0)
                                            ),
                                        ),
                                    ),

                                    ##### Is Interacting With Someone Else #####
                                    owyl.sequence(
                                        self.is_random_greater_than(val1="newRandom", val2=0.5),
                                        self.glance_at_lost_face()
                                    ),

                                    ##### Does Nothing #####
                                    owyl.sequence(
                                        self.does_nothing()
                                    )
                                ),
                                self.clear_lost_face_target()
                            ),

                            ##### People Interaction #####
                            owyl.sequence(
                                self.is_face_target(),
                                owyl.selector(
                                    ##### Start A New Interaction #####
                                    owyl.sequence(
                                        owyl.selector(
                                            self.is_not_interacting_with_someone(),
                                            owyl.sequence(
                                                self.is_more_than_one_face_target(),
                                                self.is_time_to_change_face_target(limit=10)
                                            )
                                        ),
                                        self.stop_interaction(),
                                        self.select_a_face_target(),
                                        self.record_start_time(variable="interact_with_face_target_since"),
                                        self.interact_with_face_target(id="current_face_target", min_duration=2.0, max_duration=4.0)
                                    ),

                                    ##### Glance At Other Faces & Continue With The Last Interaction #####
                                    owyl.sequence(
                                        self.select_a_glance_target(),
                                        self.glance_at(id="current_glance_target"),
                                        self.interact_with_face_target(id="current_face_target", min_duration=2.0, max_duration=4.0)
                                    )
                                )
                            ),

                            ##### Nothing Interesting Is Happening #####
                            owyl.sequence(
                                self.update_emotion(variable="boredom_engagement", min=0.7, max=0.9),
                                owyl.selector(
                                    ##### Is Not Sleeping #####
                                    owyl.sequence(
                                        self.is_not_sleeping(),
                                        owyl.selector(
                                            ##### Go To Sleep #####
                                            owyl.sequence(
                                                self.is_random_smaller_than(val1="newRandom_plus_boredom", val2=0.6),
                                                self.record_start_time(variable="sleep_since"),
                                                self.go_to_sleep(min_duration=5.0, max_duration=10.0)
                                            ),

                                            ##### If Interruptions -> Wake Up #####
                                            owyl.sequence(
                                                self.is_sleeping(),
                                                self.wake_up(),
                                                self.update_emotion(variable="boredom_engagement", min="2.2", max="2.6")
                                            ),

                                            ##### Search For Attention #####
                                            owyl.sequence(
                                                self.search_for_attention(min_duration=5.0, max_duration=10.0)
                                            )
                                        )
                                    ),

                                    ##### Is Sleeping #####
                                    owyl.selector(
                                        ##### Wake Up #####
                                        owyl.sequence(
                                            self.is_random_greater_than(val1="newRandom", val2=0.5),
                                            self.is_time_to_wake_up(limit=60),
                                            self.wake_up(),
                                            self.update_emotion(variable="boredom_engagement", min="2.2", max="2.6")
                                        ),

                                        ##### Continue To Sleep #####
                                        owyl.sequence(
                                            self.go_to_sleep(min_duration=5.0, max_duration=10.0)
                                        ),

                                        ##### If Interruptions -> Wake Up #####
                                        owyl.sequence(
                                            self.is_sleeping(),
                                            self.wake_up(),
                                            self.update_emotion(variable="boredom_engagement", min="2.2", max="2.6")
                                        )
                                    )
                                )
                            )
                        )
                    )
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

    @owyl.taskmethod
    def sync_variables(self, **kwargs):
        if not self.blackboard["is_TrackDev_on"]:
            self.tracking_mode_pub.publish("TrackDev")
            self.blackboard["is_TrackDev_on"] = True
        self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
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
        self.blackboard[kwargs["variable"]] *= random.uniform(kwargs["min"], kwargs["max"])
        yield True

    @owyl.taskmethod
    def is_random_greater_than(self, **kwargs):
        if kwargs["val1"] == "newRandom":
            self.blackboard["random"] = random.random()
        if self.blackboard["random"] > kwargs["val2"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_random_smaller_than(self, **kwargs):
        if kwargs["val1"] == "newRandom":
            self.blackboard["random"] = random.random()
        elif kwargs["val1"] == "newRandom_plus_boredom":
            self.blackboard["random"] = random.random() + self.blackboard["boredom_engagement"]
        if self.blackboard["random"] < kwargs["val2"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_random_in_between(self, **kwargs):
        if kwargs["val"] == "newRandom":
            self.blackboard["random"] = random.random()
        if kwargs["greater"] < self.blackboard["random"] < kwargs["smaller"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_arrived(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if self.blackboard["new_face"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_someone_left(self, **kwargs):
        self.blackboard["is_interruption"] = False
        if len(self.blackboard["lost_face"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_interacting_with_someone(self, **kwargs):
        if self.blackboard["current_face_target"]:
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
            yield True
        else:
            yield False

    @owyl.taskmethod
    def was_interacting_with_that_person(self, **kwargs):
        if self.blackboard["current_face_target"] == self.blackboard["lost_face"]:
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
        if (time.time() - self.blackboard["interact_with_face_target_since"]) >= kwargs["limit"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_time_to_wake_up(self, **kwargs):
        if (time.time() - self.blackboard["sleep_since"]) >= kwargs["limit"]:
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
    def assign_face_target(self, **kwargs):
        self.blackboard["current_face_target"] = self.blackboard["face_targets"][0]
        yield True

    @owyl.taskmethod
    def select_a_face_target(self, **kwargs):
        self.blackboard["current_face_target"] = random.choice(self.blackboard["face_targets"])
        yield True

    @owyl.taskmethod
    def select_a_glance_target(self, **kwargs):
        target = random.choice(self.blackboard["face_targets"])
        if len(self.blackboard["face_targets"]) > 1:
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
        face_id = self.blackboard[kwargs["id"]]
        duration = random.uniform(kwargs["min_duration"], kwargs["max_duration"])
        interval = 0.01
        self.face_id_pub(face_id)
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        # if duration <= 0:
        #     self.tracking_mode_pub.publish("Dummy")
        #     self.blackboard["is_TrackDev_on"] = False
        yield True

    @owyl.taskmethod
    def stop_interaction(self, **kwargs):
        self.blackboard["current_face_target"] = ""
        if self.blackboard["is_TrackDev_on"]:
            self.tracking_mode_pub.publish("Dummy")
            self.blackboard["is_TrackDev_on"] = False
        yield True

    @owyl.taskmethod
    def glance_at(self, **kwargs):
        face_id = self.blackboard[kwargs["id"]]
        # TODO: Construct and publish the TrackingAction Message
        # self.tracking_action_pub.publish()
        yield True

    @owyl.taskmethod
    def glance_at_new_face(self, **kwargs):
        face_id = self.blackboard["new_face"]
        # TODO: Construct and publish the TrackingAction Message
        # self.tracking_action_pub.publish()
        yield True

    @owyl.taskmethod
    def glance_at_lost_face(self, **kwargs):
        face_id = self.blackboard["lost_face"]
        # TODO: Construct and publish the TrackingAction Message
        # self.tracking_action_pub.publish()
        yield True

    @owyl.taskmethod
    def show_expression(self, **kwargs):
        expression = kwargs["expression"]
        duration = random.uniform(kwargs["min_duration"], kwargs["max_duration"])
        # TODO: Construct and publish the basic_head_api/MakeFaceExpr Message
        # self.emotion_pub.publish()
        yield True

    @owyl.taskmethod
    def search_for_attention(self, **kwargs):
        duration = random.uniform(kwargs["min_duration"], kwargs["max_duration"])
        interval = 0.01
        self.tracking_mode_pub.publish("LookAround")
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        yield True

    @owyl.taskmethod
    def go_to_sleep(self, **kwargs):
        self.blackboard["is_sleeping"] = True
        duration = random.uniform(kwargs["min_duration"], kwargs["max_duration"])
        interval = 0.01
        #TODO: Topic for sleep
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                yield False
                break
        yield True

    @owyl.taskmethod
    def wake_up(self, **kwargs):
        self.blackboard["is_sleeping"] = False
        self.blackboard["sleep_since"] = 0.0
        #TODO: Topic for waking up
        yield True

    @owyl.taskmethod
    def say(self, **kwargs):
        utterance = kwargs["utterance"]
        # TODO: See if needed
        yield True

    @owyl.taskmethod
    def nod(self, **kwargs):
        # TODO: See if needed
        yield True

    @owyl.taskmethod
    def clear_new_face_target(self, **kwargs):
        self.blackboard["new_face"] = ""
        yield True

    @owyl.taskmethod
    def clear_lost_face_target(self, **kwargs):
        self.blackboard["lost_face"] = ""
        yield True

    def tracking_event_callback(self, data):
        self.blackboard["is_interruption"] = True
        if data.event == "new_face":
            self.blackboard["new_face"] = data.parameter
            self.blackboard["background_face_targets"].append(data.parameter)
        elif data.event == "exit":
            self.blackboard["lost_face"] = data.parameter
            self.blackboard["background_face_targets"].remove(data.parameter)

if __name__ == "__main__":
    rospy.init_node("Eva_Behavior")
    tree = Tree()
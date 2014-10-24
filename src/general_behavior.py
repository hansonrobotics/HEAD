#!/usr/bin/python
import owyl
from owyl import blackboard
import rospy
import roslib
import random
import time
from std_msgs.msg import String
from eva_behavior.msg import event
from eva_behavior.msg import tracking_action
from eva_behavior.msg import emotion


class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.blackboard["sadness_happiness"] = 0.5
        self.blackboard["irritation_amusement"] = 0.5
        self.blackboard["confusion_comprehension"] = 0.5
        self.blackboard["boredom_engagement"] = 0.5
        self.blackboard["recoil_surprise"] = 0.5
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
        self.blackboard["blender_mode"] = ""
        self.blackboard["is_scripted_performance_system_on"] = False
        self.blackboard["random"] = 0.0
        rospy.Subscriber("scripted", String, self.scripted_performance_system_callback)
        rospy.Subscriber("tracking_event", event, self.tracking_event_callback)
        self.tracking_mode_pub = rospy.Publisher("/cmd_blendermode", String, queue_size=1)
        self.tracking_action_pub = rospy.Publisher("tracking_action", tracking_action, queue_size=1)
        self.emotion_pub = rospy.Publisher("emotion", emotion, queue_size=1)
        self.tree = self.build_tree()
        while True:
            self.tree.next()

    def build_tree(self):
        eva_behavior_tree = \
            owyl.parallel(
                ############### Tree: General Behaviors ###############
                owyl.repeatAlways(
                    owyl.selector(
                        owyl.sequence(
                            self.is_scripted_performance_system_off(),
                            self.sync_variables(),
                            ########## Main Events ##########
                            owyl.selector(
                                ##### When Someone Arrived #####
                                owyl.sequence(
                                    self.is_someone_arrived(),
                                    self.set_emotion(variable="boredom_engagement", value=0.5),
                                    self.test(str="----- Someone Arrived!"),
                                    owyl.selector(
                                        ##### Were No People In The Scene #####
                                        owyl.sequence(
                                            self.were_no_people_in_the_scene(),
                                            self.test(str="----- Were No People In The Scene!"),
                                            self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.2, max=1.4),
                                            self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.2, max=1.4),
                                            self.assign_face_target(variable="current_face_target", value="new_face"),
                                            self.record_start_time(variable="interact_with_face_target_since"),
                                            self.interact_with_face_target(id="current_face_target", min_duration=5.0, max_duration=10.0)
                                        ),

                                        ##### Is Interacting With Someone #####
                                        owyl.sequence(
                                            self.is_interacting_with_someone(),
                                            self.test(str="----- Is Interacting With Someone!"),
                                            self.is_random_greater_than(val1="newRandom", val2=0.5),
                                            self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=1.05, max=1.1),
                                            self.update_emotion(variable="boredom_engagement", lower_limit=0.0, min=1.05, max=1.1),
                                            self.glance_at_new_face()
                                        ),

                                        ##### Does Nothing #####
                                        owyl.sequence(
                                            self.test(str="----- Ignoring The New Face!"),
                                            self.does_nothing()
                                        )
                                    ),
                                    self.clear_new_face_target()
                                ),

                                ##### When Someone Left #####
                                owyl.sequence(
                                    self.is_someone_left(),
                                    self.test(str="----- Someone Left!"),
                                    owyl.selector(
                                        ##### Was Interacting With That Person #####
                                        owyl.sequence(
                                            self.was_interacting_with_that_person(),
                                            self.test(str="----- Was Interacting With That Person!"),
                                            owyl.selector(
                                                ##### Confused #####
                                                owyl.sequence(
                                                    self.is_random_smaller_than(val1="newRandom", val2=0.4),
                                                    self.update_emotion(variable="confusion_comprehension", lower_limit=0.0, min=0.4, max=0.6),
                                                    self.update_emotion(variable="recoil_surprise", lower_limit=0.0, min=1.05, max=1.1),
                                                    self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.update_emotion(variable="irritation_amusement", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.show_expression(expression="confused", min_duration=1.0, max_duration=3.0)
                                                ),

                                                ##### Surprise #####
                                                owyl.sequence(
                                                    self.is_random_in_between(val="random", greater=0.4, smaller=0.8),
                                                    self.update_emotion(variable="recoil_surprise", lower_limit=0.0, min=1.4, max=1.6),
                                                    self.update_emotion(variable="confusion_comprehension", lower_limit=0.0, min=0.4, max=0.6),
                                                    self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.update_emotion(variable="irritation_amusement", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.show_expression(expression="surprise", min_duration=1.0, max_duration=3.0)
                                                ),

                                                ##### Sad #####
                                                owyl.sequence(
                                                    self.update_emotion(variable="sadness_happiness", lower_limit=0.0, min=0.4, max=0.6),
                                                    self.update_emotion(variable="recoil_surprise", lower_limit=0.0, min=1.05, max=1.1),
                                                    self.update_emotion(variable="confusion_comprehension", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.update_emotion(variable="irritation_amusement", lower_limit=0.0, min=0.95, max=1.0),
                                                    self.show_expression(expression="sad", min_duration=1.0, max_duration=3.0)
                                                ),
                                            ),
                                        ),

                                        ##### Is Interacting With Someone Else #####
                                        owyl.sequence(
                                            self.test(str="----- Is Interacting With Someone Else!"),
                                            self.is_random_greater_than(val1="newRandom", val2=0.5),
                                            self.glance_at_lost_face()
                                        ),

                                        ##### Does Nothing #####
                                        owyl.sequence(
                                            self.test(str="----- Ignoring The Lost Face!"),
                                            self.does_nothing()
                                        )
                                    ),
                                    self.clear_lost_face_target()
                                ),

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
                                                    self.is_time_to_change_face_target(limit=10)
                                                )
                                            ),
                                            self.test(str="----- Time To Start A New Interaction!"),
                                            # self.stop_interaction(),
                                            self.select_a_face_target(),
                                            self.record_start_time(variable="interact_with_face_target_since"),
                                            self.interact_with_face_target(id="current_face_target", min_duration=3.0, max_duration=6.0)
                                        ),

                                        ##### Glance At Other Faces & Continue With The Last Interaction #####
                                        owyl.sequence(
                                            self.test(str="----- Continue The Interaction"),
                                            owyl.selector(
                                                owyl.sequence(
                                                    self.is_more_than_one_face_target(),
                                                    self.is_random_greater_than(val1="newRandom", val2=0.3),
                                                    self.select_a_glance_target(),
                                                    self.glance_at(id="current_glance_target")
                                                ),
                                                self.does_nothing()
                                            ),
                                            self.interact_with_face_target(id="current_face_target", min_duration=3.0, max_duration=6.0)
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
                                                    self.is_random_smaller_than(val1="newRandom_plus_boredom", val2=0.4),
                                                    self.record_start_time(variable="sleep_since"),
                                                    self.test(str="----- Go To Sleep!"),
                                                    self.go_to_sleep(min_duration=5.0, max_duration=10.0)
                                                ),

                                                ##### Feeling Sleepy #####

                                                ##### Search For Attention #####
                                                owyl.sequence(
                                                    self.test(str="----- Search For Attention!"),
                                                    self.search_for_attention(min_duration=5.0, max_duration=10.0)
                                                )
                                            )
                                        ),

                                        ##### Is Sleeping #####
                                        owyl.selector(
                                            ##### Wake Up #####
                                            owyl.sequence(
                                                self.is_random_greater_than(val1="newRandom", val2=0.5),
                                                self.is_time_to_wake_up(limit=30),
                                                self.wake_up(),
                                                self.test(str="----- Wake Up!"),
                                                self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
                                            ),

                                            ##### Waking Up #####

                                            ##### Continue To Sleep #####
                                            owyl.sequence(
                                                self.test(str="----- Continue To Sleep!"),
                                                self.go_to_sleep(min_duration=1.0, max_duration=5.0)
                                            )
                                        )
                                    ),

                                    ##### If Interruption && Sleeping -> Wake Up #####
                                    owyl.sequence(
                                        self.is_interruption(),
                                        self.is_sleeping(),
                                        self.wake_up(),
                                        self.test(str="----- Interruption: Wake Up!"),
                                        self.update_emotion(variable="boredom_engagement", lower_limit=0.3, min=1.5, max=2.0)
                                    )
                                )
                            )
                        ),

                        ############### Scripted Performance System ###############
                        owyl.sequence(
                            self.is_scripted_performance_system_on(),
                            self.start_scripted_performance_system()
                        )
                    )
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

    @owyl.taskmethod
    def test(self, **kwargs):
        print kwargs["str"]
        yield True

    @owyl.taskmethod
    def sync_variables(self, **kwargs):
        self.blackboard["face_targets"] = self.blackboard["background_face_targets"]
        print "\n========== Emotion Space =========="
        print "sadness_happiness: " + str(self.blackboard["sadness_happiness"])[:5]
        print "irritation_amusement: " + str(self.blackboard["irritation_amusement"])[:5]
        print "confusion_comprehension: " + str(self.blackboard["confusion_comprehension"])[:5]
        print "boredom_engagement: " + str(self.blackboard["boredom_engagement"])[:5]
        print "recoil_surprise: " + str(self.blackboard["recoil_surprise"])[:5]
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
            self.blackboard["current_face_target"] = ""
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
        if self.blackboard["interact_with_face_target_since"] > 0 and (time.time() - self.blackboard["interact_with_face_target_since"]) >= kwargs["limit"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def is_time_to_wake_up(self, **kwargs):
        if self.blackboard["sleep_since"] > 0 and (time.time() - self.blackboard["sleep_since"]) >= kwargs["limit"]:
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
        if self.blackboard['blender_mode'] != 'TrackDev':
            self.tracking_mode_pub.publish("TrackDev")
            self.blackboard['blender_mode'] = 'TrackDev'
            time.sleep(0.1)
        print "----- Is Interacting With: " + self.blackboard[kwargs["id"]]
        face_id = self.blackboard[kwargs["id"]]
        duration = random.uniform(kwargs["min_duration"], kwargs["max_duration"])
        interval = 0.01
        tracking_action.target = face_id
        tracking_action.action = "track"
        tracking_action.params = ""
        self.tracking_action_pub.publish(tracking_action)
        while duration > 0:
            time.sleep(interval)
            duration -= interval
            if self.blackboard["is_interruption"]:
                break
        yield True

    @owyl.taskmethod
    def glance_at(self, **kwargs):
        print "----- Glancing At " + self.blackboard[kwargs["id"]]
        face_id = self.blackboard[kwargs["id"]]
        tracking_action.target = "/faces/" + face_id
        tracking_action.action = "glance"
        tracking_action.params = ""
        self.tracking_action_pub.publish(tracking_action)
        yield True

    @owyl.taskmethod
    def glance_at_new_face(self, **kwargs):
        print "----- Glancing At The New Face " + self.blackboard["new_face"]
        face_id = self.blackboard["new_face"]
        tracking_action.target = "/faces/" + face_id
        tracking_action.action = "glance"
        tracking_action.params = ""
        self.tracking_action_pub.publish(tracking_action)
        yield True

    @owyl.taskmethod
    def glance_at_lost_face(self, **kwargs):
        print "----- Glancing At The Lost Face " + self.blackboard["lost_face"]
        face_id = self.blackboard["lost_face"]
        tracking_action.target = "/faces/" + face_id
        tracking_action.action = "glance"
        tracking_action.params = ""
        self.tracking_action_pub.publish(tracking_action)
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
        if self.blackboard['blender_mode'] != 'Dummy':
            self.tracking_mode_pub.publish("Dummy")
            self.blackboard['blender_mode']= 'Dummy'
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
                break
        yield True

    @owyl.taskmethod
    def wake_up(self, **kwargs):
        self.blackboard["is_sleeping"] = False
        self.blackboard["sleep_since"] = 0.0
        #TODO: Topic for waking up
        yield True

    @owyl.taskmethod
    def clear_new_face_target(self, **kwargs):
        if not self.blackboard["is_interruption"]:
            print "----- Cleared New Face: " + self.blackboard["new_face"]
            self.blackboard["new_face"] = ""
        yield True

    @owyl.taskmethod
    def clear_lost_face_target(self, **kwargs):
        print "----- Cleared Lost Face: " + self.blackboard["lost_face"]
        self.blackboard["lost_face"] = ""
        yield True

    @owyl.taskmethod
    def start_scripted_performance_system(self, **kwargs):
        self.tracking_mode_pub.publish("Dummy")
        self.blackboard['blender_mode'] = 'Dummy'
        self.blackboard["is_TrackDev_on"] = False
        # TODO
        yield True

    def tracking_event_callback(self, data):
        self.blackboard["is_interruption"] = True
        if data.event == "new_face":
            print "<< Interruption >> New Face Detected: " + data.param
            self.blackboard["new_face"] = data.param
            self.blackboard["background_face_targets"].append(self.blackboard["new_face"])
        elif data.event == "exit":
            print "<< Interruption >> Lost Face Detected: " + data.param
            self.blackboard["lost_face"] = data.param
            self.blackboard["background_face_targets"].remove(self.blackboard["lost_face"])
            # If the robot lost the new face during the initial interaction, reset new_face variable
            if self.blackboard["new_face"] == self.blackboard["lost_face"]:
                self.blackboard["new_face"] = ""

    def scripted_performance_system_callback(self, data):
        if not self.blackboard["is_scripted_performance_system_on"]:
            self.blackboard["is_interruption"] = True
            self.blackboard["is_scripted_performance_system_on"] = True
            print "Scripted Performance System is ON!"
        else:
            self.blackboard["is_interruption"] = False
            self.blackboard["is_scripted_performance_system_on"] = False
            print "Scripted Performance System is OFF!"

if __name__ == "__main__":
    rospy.init_node("Eva_Behavior")
    tree = Tree()
import owyl
from owyl import blackboard
import rospy
from std_msgs.msg import String
import time


class Tree():
    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.blackboard["face_targets"] = {"id": ["time_of_arrival", "age", "time_spent_with_target"]}
        self.blackboard["is_urgent_event"] = False
        self.blackboard["bored_since"] = 0.0
        self.blackboard["boredom_age"] = 0.0
        self.blackboard["boredom_level"] = 0.0
        self.blackboard["slept_since"] = 0.0
        self.blackboard["sleep_age"] = 0.0
        self.tree = self.build_tree()
        while True:
            self.tree.next()

    def build_tree(self):
        eva_behavior_tree = \
            owyl.parallel(
                ########## Update Variables ##########
                owyl.repeatAlways(
                    owyl.sequence(
                        self.update_ages(),
                        owyl.sequence(
                            self.is_no_tree_running(),
                            self.sync_with_background_variables(),
                            self.update_can_run_flags()
                        )
                    )
                ),

                ########## PERCEPTION ##########
                owyl.repeatAlways(
                    owyl.sequence(
                        self.can_run_perception_tree(),
                        owyl.selector(
                            ##### Main Events #####
                            owyl.sequence(
                                self.is_no_urgent_event(),
                                owyl.selector(
                                    ##### Someone Arrives In Scene #####
                                    owyl.sequence(
                                        self.is_not_sleeping(),
                                        self.is_new_face_target(),
                                        self.roll_dice_for_glancing(),
                                        owyl.selector(
                                            owyl.sequence(
                                                self.should_glance_at_target(threshold=0.5),
                                                self.glance_at_face_target(),
                                                self.wait_for(time=0.5)
                                            ),
                                            self.do_nothing()
                                        )
                                    ),
                                    ##### Someone Exits The Scene #####
                                    owyl.sequence(
                                        self.is_not_sleeping(),
                                        self.lost_face_target(),
                                        self.roll_dice_for_glancing(),
                                        owyl.selector(
                                            owyl.sequence(
                                                self.should_glance_at_target(threshold=0.5),
                                                self.glance_at_lost_face_target(),
                                                self.wait_for(time=0.5)
                                            ),
                                            self.do_nothing()
                                        )
                                    ),
                                    ##### Other Salience Events or In Your Face Event #####
                                    owyl.sequence(
                                        owyl.selector(
                                            self.is_salience_event(),
                                            self.is_in_your_face_event(),
                                        ),
                                        self.roll_dice_for_flinching(),
                                        owyl.selector(
                                            owyl.sequence(
                                                self.should_flinch_at_salience_target(threshold=0.5),
                                                self.flinch_at_salience_target(),
                                                self.wait_for(time=0.5)
                                            ),
                                            self.do_nothing()
                                        )
                                    ),
                                    ##### Interaction #####
                                    owyl.sequence(
                                        # TODO: TBC
                                        self.is_face_target(),
                                        self.interact_with_targets(),
                                        self.wait_and_update_time_spent_with_target(time=3.0),
                                    ),
                                    ##### Idling #####
                                    owyl.sequence(
                                        owyl.selector(
                                            ##### If Is Sleeping #####
                                            owyl.sequence(
                                                self.is_sleeping(),
                                                self.roll_dice_for_sleep_or_wake_up(),
                                                owyl.selector(
                                                    ##### Wake Up #####
                                                    owyl.sequence(
                                                        owyl.selector(
                                                            self.should_wake_up(threshold=0.5),
                                                            self.is_time_to_wake_up(),
                                                            self.is_new_face_target()
                                                        ),
                                                        self.wake_up(),
                                                        self.reset_sleep_and_boredom_variables()
                                                    ),
                                                    ##### Continue To Sleep #####
                                                    self.wait_and_update_sleep_variables(time=5.0)
                                                ),
                                            ),
                                            ##### If Is Not Sleeping #####
                                            owyl.sequence(
                                                self.roll_dice_for_boredom_or_sleep(),
                                                owyl.selector(
                                                    ##### Go To Sleep #####
                                                    owyl.sequence(
                                                        owyl.selector(
                                                            self.should_go_to_sleep(threshold=0.5),
                                                            self.is_very_bored()
                                                        ),
                                                        self.go_to_sleep(),
                                                        self.wait_and_update_sleep_variables(time=5.0)
                                                    ),
                                                    owyl.sequence(
                                                        ##### Boredom #####
                                                        self.search_for_attention(),
                                                        self.wait_and_update_boredom_variables(time=5.0)
                                                    )
                                                )
                                            )
                                        )
                                    )
                                )
                            ),
                            ##### Operator Interrupt System #####
                            owyl.sequence(
                                ##### If Is Sleeping #####
                                owyl.selector(
                                    owyl.sequence(
                                        self.is_sleeping(),
                                        self.wake_up(),
                                        self.reset_sleep_and_boredom_variables()
                                    ),
                                    self.do_nothing()
                                ),
                                # TODO: How should Eva react (TBC)
                                self.look_at_target(),
                                self.wait_for(time=3.0),
                                self.reset_is_urgent_event()
                            )
                        ),
                        self.perception_tree_is_finished()
                    )
                ),

                ########## UNDERSTANDING and MOTIVATION ##########
                # owyl.repeatAlways(
                #     owyl.sequence(
                #         self.can_run_motivation_tree(),
                #         owyl.selector(
                #             ##### Basic Semantic Gesture Analysis #####
                #             owyl.sequence(
                #                 self.is_no_urgent_event(),
                #                 owyl.selector(
                #                     ##### Someone Arrives In Scene #####
                #                     owyl.sequence(
                #                         self.is_new_face_target(),
                #                         owyl.sequence(
                #                             self.is_the_only_face_target(),
                #                             self.start_initial_arrival_response(),  # selection: dice roll for neutral, surprised or confused (low probability); amplitude: dice roll for amount
                #                             self.wait_for(time=0.5)
                #                         ),
                #                         self.start_arrival_consideration(),  # selection: engaged; amplitude: dice roll engagement amount (skewed towards >0.5)
                #                         self.wait_for(time=0.5),
                #                         self.start_arrival_resolution(),  # selection: happy; amplitude: increase happiness by dice roll amount
                #                         self.wait_for(time=1.0)
                #                     ),
                #                     ##### Someone Exits The Scene #####
                #                     owyl.sequence(
                #                         self.lost_face_target(),
                #                         self.start_initial_departure_response(),  # selection: dice roll for neutral, surprised or confused; amplitude: dice roll for amount
                #                         self.wait_for(time=0.5),
                #                         self.start_departure_consideration(),  # selection: engaged; amplitude: dice roll engagement amount
                #                         self.wait_for(time=0.5),
                #                         self.start_departure_conflict(),  # selection: dice roll for confusion or irritation; amplitude: dice roll for amount
                #                         self.wait_for(time=0.5),
                #                         self.start_departure_resolution(),  # selection: unhappy; amplitude: decrease happiness by dice roll amount
                #                         self.wait_for(time=1.0)
                #                     ),
                #                     ##### Other Salience Events or In Your Face Event #####
                #                     owyl.sequence(
                #                         owyl.selector(
                #                             self.is_salience_event(),
                #                             self.is_in_your_face_event(),
                #                         ),
                #                         self.start_initial_salience_response(),  # selection: dice roll for neutral, surprise/recoil or confused; amplitude: dice roll for amount
                #                         self.wait_for(time=0.5),
                #                         self.start_salience_conflict(),  # selection: dice roll for confusion or irritation/amusement; amplitude: dice roll for amount
                #                         self.wait_for(time=0.5),
                #                         self.start_salience_resolution(),  # selection: choose happy if amusement (>=0.5) or unhappy if irritated or confused (<0.5); amplitude: dice roll for amount; increase or decrease happiness per selection
                #                         self.wait_for(time=1.0)
                #                     ),
                #                     ##### Group Interaction or Look At Someone #####
                #                     owyl.sequence(
                #                         self.is_face_target(),
                #                         self.select_group_interaction_response(),  # selection: happy & engaged; amplitude: force happiness and engagement into a dynamic equilibrium that fluctuates for interest by stays above 0.5 as this ongoing group interaction is her happiness zone
                #                         self.wait_for(time=1.0)
                #                     ),
                #                     ##### Wake Up #####
                #                     owyl.sequence(
                #                         self.is_time_to_wake_up()  # With sleep_age > threshold
                #                     ),
                #                     ##### Go To Sleep #####
                #                     owyl.sequence(
                #                         self.is_very_bored()  # With boredom age && boredom level > thresholds
                #                     ),
                #                     ##### Boredom #####
                #                     owyl.sequence(
                #                         self.select_boredom_response(),  # selection: dice roll bored or happy/sad or irritated/amused; amplitude: dice roll amount
                #                         self.wait_for(time=5.0)
                #                     )
                #                 )
                #             ),
                #             ##### Operator Interrupt System #####
                #             owyl.sequence(
                #                 # TODO: How should Eva react (TBC)
                #                 self.select_response(),
                #                 self.wait_for(time=3.0)
                #             )
                #         ),
                #         self.motivation_tree_is_finished()
                #     )
                # ),

                ########## Speech ##########
                owyl.repeatAlways(
                    owyl.sequence(
                        self.can_run_speech_tree(),
                        owyl.selector(
                            ##### Say Something #####
                            owyl.sequence(
                                self.is_no_urgent_event(),
                                self.is_audio_input(),
                                owyl.selector(
                                    owyl.sequence(
                                        self.is_command(),
                                        self.execute_command()
                                    ),
                                    self.send_to_zenodial()
                                )
                            ),
                            ##### Operator Interrupt System #####
                            owyl.sequence(
                                self.is_speaking(),
                                self.stop_speaking()
                            )
                        ),
                        self.speech_tree_is_finished()
                    )
                ),

                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(eva_behavior_tree, blackboard=self.blackboard)

if __name__ == "__main__":
    rospy.init_node("Eva_Behavior")
    tree = Tree()
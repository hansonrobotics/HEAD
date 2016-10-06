define(['jquery', 'roslib', 'js-yaml', './api'], function ($, ROSLIB, jsyaml, api) {
    var ros = {
        // Current status
        connected: false,
        // Connects to ROS
        connect: function (success) {
            //Connect to rosbridge
            this.instance = api.ros = new ROSLIB.Ros({
                url: this.rosUrl()
            }).on("connection", function (e) {
                    api.getRobotName(function (name) {
                        if (name) {
                            // Finish initializing UI
                            console.log('robot: ' + name);
                            api.config.robot = name;

                            ros.initTopics();
                            ros.initServices();
                            success();
                        } else {
                            console.error('Unable to get robot name, closing connection');
                            api.ros.close();
                        }
                    });
                }).on('connection', function () {
                    $('#app-connecting').hide();
                    $('#app-pages').fadeIn();
                    api.ros.connected = true;
                }).on('close', function () {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    api.ros.connected = false;
                }).on('error', function (error) {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    api.ros.connected = false;
                    $('#app-pages').fadeIn();
                    success();
                });
        },
        initTopics: function () {
            api.logger = new ROSLIB.Topic({
                ros: api.ros,
                name: '/rosout',
                messageType: 'rosgraph_msgs/Log',
            });
            api.topics = {
                cmdTree: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/behavior_switch',
                    messageType: 'std_msgs/String'
                }),
                btMode: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/behavior_control',
                    messageType: 'std_msgs/Int32'
                }),
                speech_topic: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/speech',
                    messageType: 'chatbot/ChatMessage'
                }),
                expression: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/make_face_expr',
                    messageType: 'basic_head_api/MakeFaceExpr'
                }),
                pointHeadTopic: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/point_head',
                    messageType: 'basic_head_api/PointHead'
                }),
                available_gestures: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/available_gestures'
                }),
                available_emotion_states: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/available_emotion_states'
                }),
                available_soma_states: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/available_soma_states'
                }),
                set_gesture: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_gesture',
                    messageType: 'blender_api_msgs/SetGesture'
                }),
                set_emotion_state: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_emotion_state',
                    messageType: 'blender_api_msgs/EmotionState'
                }),
                set_soma_state: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_soma_state',
                    messageType: 'blender_api_msgs/SomaState'
                }),
                set_head_rotation: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_head_rotation',
                    messageType: 'std_msgs/Float32'
                }),
                play_animation: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/play_animation'
                }),
                speech_active: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/speech_events',
                    messageType: 'std_msgs/String'
                }),
                execute_scripts: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/execute_script',
                    messageType: 'std_msgs/String'
                }),
                scripts_available: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/scripts',
                    messageType: 'std_msgs/String'
                }),
                set_face_target: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_face_target'
                }),
                set_gaze_target: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_gaze_target'
                }),
                selected_tts_mux: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/tts_mux/selected'
                }),
                chatbot_responses: {
                    'default': new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/chatbot_responses',
                        messageType: 'std_msgs/String'
                    }),
                    en: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/chatbot_responses_en',
                        messageType: 'std_msgs/String'
                    }),
                    zh: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/chatbot_responses_zh',
                        messageType: 'std_msgs/String'
                    })
                },
                web_responses: {
                    'default': new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/web_responses',
                        messageType: 'std_msgs/String'
                    }),
                    en: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/web_responses_en',
                        messageType: 'std_msgs/String'
                    }),
                    zh: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/web_responses_zh',
                        messageType: 'std_msgs/String'
                    })
                },
                tts: {
                    'default': new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts',
                        messageType: 'std_msgs/String'
                    }),
                    en: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts_en',
                        messageType: 'std_msgs/String'
                    }),
                    zh: new ROSLIB.Topic({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts_zh',
                        messageType: 'std_msgs/String'
                    })
                },
                face_locations: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/camera/face_locations',
                    messageType: 'pi_face_tracker/Faces'
                }),
                set_look_at_face: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/camera/face_event',
                    messageType: 'pi_face_tracker/FaceEvent'
                }),
                get_look_at_face: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/look_at_face',
                    messageType: 'std_msgs/Int32'
                }),
                voice: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/voice',
                    messageType: 'std_msgs/String'
                }),
                chat_events: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/chat_events',
                    messageType: 'std_msgs/String'
                }),
                performance_events: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/performances/events',
                    messageType: 'performances/Event'
                }),
                set_animation_mode: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/set_animation_mode',
                    messageType: 'std_msgs/Uint8'
                }),
                get_animation_mode: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/get_animation_mode',
                    messageType: 'std_msgs/Uint8'
                }),
                bug_log: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/webui/log/bugs',
                    messageType: 'std_msgs/String'
                }),
                chat_log: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/webui/log/chat',
                    messageType: 'std_msgs/String'
                }),
                tts_control: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/tts_control',
                    messageType: 'std_msgs/String'
                }),
                listen_node_input: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/nodes/listen/input',
                    messageType: 'std_msgs/String'
                })
            };
        },
        initServices: function () {
            api.services = {
                headPauMux: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/head_pau_mux/select',
                    serviceType: 'topic_tools/MuxSelect'
                }),
                neckPauMux: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/neck_pau_mux/select',
                    serviceType: 'topic_tools/MuxSelect'
                }),
                expressionList: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/valid_face_exprs',
                    serviceType: 'basic_head_api/ValidFaceExprs'
                }),
                topicsForType: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/rosapi/topics_for_type',
                    serviceType: 'rosapi/TopicsForType'
                }),
                tts_length: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/tts_length',
                    messageType: 'tts/TTSLength'
                }),
                get_animation_length: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/blender_api/get_animation_length',
                    messageType: 'blender_api_msgs/GetAnimationLength'
                }),
                get_kf_animation_length: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/animation_length',
                    messageType: 'basic_head_api/AnimationLength'
                }),
                get_configurable_nodes: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/webui/get_configurable_nodes',
                    messageType: 'webui/ConfigurableNodes'
                }),
                get_node_configuration: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/webui/get_configuration',
                    messageType: 'webui/NodeConfiguration'
                }),
                get_node_description: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/webui/get_description',
                    messageType: 'webui/NodeDescription'
                }),
                set_dxl_torque: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/set_dxl_torque',
                    messageType: 'dynamixel_controllers/TorqueEnable'
                }),
                get_motor_states: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/get_motor_states',
                    messageType: 'webui/MotorStates'
                }),
                performances: {
                    load_sequence: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/load_sequence',
                        messageType: 'performances/LoadSequence'
                    }),
                    load_performance: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/load_performance',
                        messageType: 'performances/LoadPerformance'
                    }),
                    current: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/current',
                        messageType: 'performances/Current'
                    }),
                    run: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/run',
                        messageType: 'performances/Run'
                    }),
                    stop: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/stop',
                        messageType: 'performances/Stop'
                    }),
                    pause: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/pause',
                        messageType: 'performances/Pause'
                    }),
                    resume: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/performances/resume',
                        messageType: 'performances/Resume'
                    })
                },
                tts_select: {
                    'default': new ROSLIB.Service({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts_select',
                        messageType: 'topic_tools/MuxSelect'
                    }),
                    'en': new ROSLIB.Service({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts_en_select',
                        messageType: 'topic_tools/MuxSelect'
                    }),
                    'zh': new ROSLIB.Service({
                        ros: api.ros,
                        name: '/' + api.config.robot + '/tts_zh_select',
                        messageType: 'topic_tools/MuxSelect'
                    })
                },
                chatbot: {
                    bot_names: new ROSLIB.Service({
                        ros: api.ros,
                        name: '/webui/chatbot_controller/bot_names',
                        messageType: 'webui/Json'
                    })
                }
            };
        },
        rosUrl: function () {
            if (window.location.protocol != "https:") {
                return "ws://" + document.domain + ":9090";
            } else {
                return "wss://" + document.domain + ":9094";
            }
        }
    };

    return ros;
});

define(['jquery', 'roslib', 'jsyaml', './api'], function ($, ROSLIB, jsyaml, api) {
    var ros = {
        // Current status
        connected: false,
        // Connects to ROS
        connect: function (success) {
            //Connect to rosbridge
            api.ros = new ROSLIB.Ros({
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
            api.topics = {
                cmdTree: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/behavior_switch',
                    messageType: 'std_msgs/String'
                }),
                speech_topic: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/chatbot_speech',
                    messageType: 'chatbot/ChatMessage'
                }),
                chat_responses: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/chatbot_responses',
                    messageType: 'std_msgs/String'
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
                chatbot_responses: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/chatbot_responses',
                    messageType: 'std_msgs/String'
                }),
                face_locations: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/camera/face_locations',
                    messageType: 'pi_face_tracker/Faces'
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
                    messageType: '﻿blender_api_msgs/GetAnimationLength'
                })
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

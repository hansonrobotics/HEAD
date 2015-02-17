define(['jquery', 'roslib', 'jsyaml', './api'], function ($, ROSLIB, jsyaml, api) {
    var ros = {
        init: function (success) {
            this.loadMotorConfig(function () {
                ros.connect(success);
            });
        },
        loadMotorConfig: function (success) {
            $.ajax({
                url: "config/motors.yml",
                dataType: "text",
                cache: false,
                success: function (motorsConfig) {
                    api.config.motors = [];

                    if (motorsConfig != null)
                        api.config.motors = api.config.motors.concat(jsyaml.load(motorsConfig));

                    success();
                }
            });
        },
        connect: function (success) {
            //Connect to rosbridge
            api.ros = new ROSLIB.Ros({
                url: this.rosUrl()
            }).on("connection", function (e) {
                    // call the success callback
                    success();

                    api.setDefaultMotorValues();
                }).on('connection', function () {
                    $('#app-connecting').hide();
                    $('#app-pages').fadeIn();
                }).on('close', function () {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    $('#app-title').html('');
                }).on('error', function (error) {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    $('#app-title').html('');
                });

            this.initTopics();
            this.initServices();
        },
        initTopics: function () {
            api.topics = {
                cmdBlender: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/cmd_blendermode',
                    messageType: 'std_msgs/String'
                }),
                cmdBllink: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/cmd_blink',
                    messageType: 'std_msgs/String'
                }),
                cmdTree: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/behavior_switch',
                    messageType: 'std_msgs/String'
                }),
                speech_topic: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/chatbot_speech',
                    messageType: 'chatbot/ChatMessage'
                }),
                chat_responses: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/chatbot_responses',
                    messageType: 'std_msgs/String'
                }),
                expression: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/make_coupled_face_expr',
                    messageType: 'basic_head_api/MakeCoupledFaceExpr'
                }),
                pointHeadTopic: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/point_head',
                    messageType: 'basic_head_api/PointHead'
                }),
                "/fritz/left/command": new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/left/command',
                    messageType: 'ros_pololu_servo/MotorCommand'
                }),
                "/fritz/right/command": new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/right/command',
                    messageType: 'ros_pololu_servo/MotorCommand'
                }),
                animations: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/cmd_animations',
                    messageType: 'std_msgs/String'
                }),
                neck0: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/base_controller/command',
                    messageType: 'std_msgs/Float64'
                }),
                neck1: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/base_right_controller/command',
                    messageType: 'std_msgs/Float64'
                }),
                neck2: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/base_left_controller/command',
                    messageType: 'std_msgs/Float64'
                }),
                neck3: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/neck_right_controller/command',
                    messageType: 'std_msgs/Float64'
                }),
                neck4: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/fritz/neck_left_controller/command',
                    messageType: 'std_msgs/Float64'
                }),
                available_gestures: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/available_gestures'
                }),
                available_emotion_states: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/blender_api/available_emotion_states'
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
                })
            };
        },
        initServices: function () {
            api.services = {
                headPauMux: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/fritz/head_pau_mux/select',
                    serviceType: 'topic_tools/MuxSelect'
                }),
                neckPauMux: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/fritz/neck_pau_mux/select',
                    serviceType: 'topic_tools/MuxSelect'
                }),
                expressionList: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/valid_coupled_face_exprs',
                    serviceType: 'basic_head_api/ValidCoupledFaceExprs'
                }),
                topicsForType: new ROSLIB.Service({
                    ros: api.ros,
                    name: '/rosapi/topics_for_type',
                    serviceType: 'rosapi/TopicsForType'
                })
            };
        },
        rosUrl: function () {
            if (window.location.protocol != "https:") {
                return "ws://" + document.domain + ":9090";
            } else {
                return "wss://" + document.domain + ":9092";
            }
        },
        getMotorConfig: function (name) {
            var motorConf = api.config.motors;

            for (var i = 0; i < motorConf.length; i++) {
                if (motorConf[i].name == name)
                    return motorConf[i];
            }
        }
    };

    return ros;
});

define(['jquery', 'roslib', 'jsyaml', './api'], function ($, ROSLIB, jsyaml, api) {
    var ros = {
        loadMotorConfig: function (success) {
            $.ajax({
                url: "config/motors.yml",
                dataType: "text",
                cache: false,
                success: function (motorsConfig) {
                    api.config.motors = [];

                    if (motorsConfig != null)
                        api.config.motors = api.config.motors.concat(jsyaml.load(motorsConfig));

                    success(api.config.motors);
                }
            });
        },
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
                }).on('close', function () {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    $('#app-title').html('');
                }).on('error', function (error) {
                    $('#notifications .label').hide();
                    $('#app-connection-error').show();
                    $('#app-title').html('');
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
                play_animation: new ROSLIB.Topic({
                    ros: api.ros,
                    name: '/' + api.config.robot + '/play_animation'
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
                })
            };
        },
        rosUrl: function () {
            if (window.location.protocol != "https:") {
                return "ws://" + document.domain + ":9090";
            } else {
                return "wss://" + document.domain + ":9094";
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

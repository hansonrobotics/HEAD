RosUI.ros = {
    config: {
        robotname: "fritz"
    },
    topics: {},
    init: function (success) {
        RosUI.ros.loadMotorConfig(function() {
            RosUI.ros.connect(success);
        });
    },
    loadMotorConfig: function(success) {
        var motorConfig = $.ajax({
            url: "config/motors.yml",
            dataType: "text",
            cache: false
        });

        var pololuMotorConfig = $.ajax({
            url: "config/pololu_motors.yml",
            dataType: "text",
            cache: false
        });

        $.when(motorConfig, pololuMotorConfig).done(function(motorsResponse, pololuMotorsResponse) {
            console.log(motorsResponse);
            console.log(pololuMotorsResponse);

            var motorsConfig = jsyaml.load(motorsResponse[0]);
            var pololuMotorsConfig = jsyaml.load(pololuMotorsResponse[0]);

            RosUI.ros.config.motors = [];

            if (motorsConfig != null)
                RosUI.ros.config.motors = RosUI.ros.config.motors.concat(motorsConfig);

            // allow editing for pololu motors
            if (pololuMotorsConfig != null) {
                $.each(pololuMotorsConfig, function () {
                    this.editable = true;
                });

                RosUI.ros.config.motors = RosUI.ros.config.motors.concat(pololuMotorsConfig);
            }

            success();
        });
    },
    connect: function (success) {
        //Connect to rosbridge
        RosUI.ros.ros = new ROSLIB.Ros({
            url: RosUI.ros.rosUrl()
        }).on("connection", function (e) {
                // call the success callback
                success();

                RosUI.api.setDefaultMotorValues();
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

        RosUI.ros.initTopics();
        RosUI.ros.initServices();
    },
    initTopics: function () {
        RosUI.ros.topics = {
            cmdBlender: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/cmd_blendermode',
                messageType: 'std_msgs/String'
            }),
            cmdBllink: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/cmd_blink',
                messageType: 'std_msgs/String'
            }),
            cmdTree: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/behavior_switch',
                messageType: 'std_msgs/String'
            }),
            speech_topic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/chatbot_speech',
                messageType: 'chatbot/ChatMessage'
            }),
            chat_responses: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/chatbot_responses',
                messageType: 'std_msgs/String'
            }),
            expression: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/make_coupled_face_expr',
                messageType: 'basic_head_api/MakeCoupledFaceExpr'
            }),
            pointHeadTopic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/point_head',
                messageType: 'basic_head_api/PointHead'
            }),
            left: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/left/command',
                messageType: 'ros_pololu_servo/MotorCommand'
            }),
            right: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/right/command',
                messageType: 'ros_pololu_servo/MotorCommand'
            }),
            animations: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/cmd_animations',
                messageType: 'std_msgs/String'
            }),
            neck0: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/base_controller/command',
                messageType: 'std_msgs/Float64'
            }),
            neck1: new ROSLIB.Topic({
              ros: RosUI.ros.ros,
              name: '/fritz/base_right_controller/command',
              messageType: 'std_msgs/Float64'
            }),
            neck2: new ROSLIB.Topic({
              ros: RosUI.ros.ros,
              name: '/fritz/base_left_controller/command',
              messageType: 'std_msgs/Float64'
            }),
            neck3: new ROSLIB.Topic({
              ros: RosUI.ros.ros,
              name: '/fritz/neck_right_controller/command',
              messageType: 'std_msgs/Float64'
            }),
            neck4: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/fritz/neck_left_controller/command',
                messageType: 'std_msgs/Float64'
            }),
            available_gestures: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/available_gestures'
            }),
            available_emotion_states: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/available_emotion_states'
            }),
            set_gesture: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/set_gesture',
                messageType: 'blender_api_msgs/SetGesture'
            }),
            set_emotion_state: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/blender_api/set_emotion_state',
                messageType: 'blender_api_msgs/EmotionState'
            })
        };
    },
    initServices: function () {
        RosUI.ros.services = {
            headPauMux: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/fritz/head_pau_mux/select',
                serviceType: 'topic_tools/MuxSelect'
            }),
            neckPauMux: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/fritz/neck_pau_mux/select',
                serviceType: 'topic_tools/MuxSelect'
            }),
            expressionList: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/valid_coupled_face_exprs',
                serviceType: 'basic_head_api/ValidCoupledFaceExprs'
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
        var motorConf = RosUI.ros.config.motors;

        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name == name)
                return motorConf[i];
        }
    }
};

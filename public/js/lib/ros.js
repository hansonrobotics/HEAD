RosUI.ros = {
    config: {
        robotname: "fritz"
    },
    topics: {},
    init: function (success) {
        $.ajax({
            url: "motors.yml",
            dataType: "text",

            success: function (data) {
                RosUI.ros.config.motors = jsyaml.load(data);
                RosUI.ros.connect(success);
            }
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
            })

        }
    },
    initServices: function () {
        RosUI.ros.services = {
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

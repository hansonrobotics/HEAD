RosUI.ros = {
    config: {
        robotname: "dmitry"
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
            url: RosUI.ros.ros_url()
        }).on("connection", function (e) {
                // call the success callback
                success();

                RoboInterface.$.trigger("connection");
                RoboInterface.sendDefaultMotorCmds();
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

        RosUI.ros.init_topics();
        RosUI.ros.init_services();
    },
    init_topics: function () {
        //Publish topic - to be deleted
        RoboInterface.motorCmdTopic = new ROSLIB.Topic({
            ros: RosUI.ros.ros,
            name: '/dmitry/pololu/cmd_pololu',
            messageType: 'ros_pololu_servo/servo_pololu'
        });

        RoboInterface.motortopicParams = {
            face: {
                ros: RosUI.ros.ros,
                name: '/dmitry/dmitry_face/cmd_pololu',
                messageType: 'ros_pololu_servo/servo_pololu'
            },
            eyes: {
                ros: RosUI.ros.ros,
                name: '/dmitry/dmitry_eyes/cmd_pololu',
                messageType: 'ros_pololu_servo/servo_pololu'
            },
            jaw: {
                ros: RosUI.ros.ros,
                name: '/dmitry/jaw_controller/command',
                messageType: 'std_msgs/Float64'
            }
        };
        // Publish and subscribe to topics
        RoboInterface.motorCmdTopics = {};
        $.each(RoboInterface.motortopicParams, function (k, p) {
            RoboInterface.motorCmdTopics[k] = new ROSLIB.Topic(p);
            RoboInterface.motorCmdTopics[k].subscribe(function (msg) {
            });
        });

        RosUI.ros.topics = {
            cmdBlender: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/cmd_blendermode',
                messageType: 'std_msgs/String'
            }),
            cmdBllink: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/dmitry/cmd_blink',
                messageType: 'std_msgs/String'
            }),
            cmdTree: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/dmitry/behavior_switch',
                messageType: 'std_msgs/String'
            }),
            speech_topic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/dmitry/chatbot_speech',
                messageType: 'chatbot/ChatMessage'
            }),
            expression: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/dmitry/make_coupled_face_expr',
                messageType: 'basic_head_api/MakeCoupledFaceExpr'
            }),
            pointHeadTopic: new ROSLIB.Topic({
                ros: RosUI.ros.ros,
                name: '/dmitry/point_head',
                messageType: 'basic_head_api/PointHead'
            })
        }
    },
    init_services: function() {
        RosUI.ros.services = {
            expression_list: new ROSLIB.Service({
                ros: RosUI.ros.ros,
                name: '/valid_coupled_face_exprs',
                serviceType: 'basic_head_api/ValidCoupledFaceExprs'
            })
        };
    },
    ros_url: function () {
        if (window.location.protocol != "https:") {
            return "ws://172.17.0.2:9090";
        } else {
            return "wss://" + document.domain + ":9092";
        }
    },
    get_motor_config: function (name) {
        var motorConf = RosUI.ros.config.motors;

        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name == name)
                return motorConf[i];
        }
    }
};

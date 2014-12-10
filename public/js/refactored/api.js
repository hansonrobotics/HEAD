RosUI.api = {
    config: {
        robotname: "dmitry"
    },
    topics: {},
    init: function (success) {
        $.ajax({
            url: "motors.yml",
            dataType: "text",

            success: function (data) {
                RosUI.api.config.motors = jsyaml.load(data);
                RosUI.api.connect(success);
            }
        });
    },
    connect: function (success) {
        //Connect to rosbridge
        RosUI.api.ros = new ROSLIB.Ros({
            url: RosUI.api.ros_url()
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

        RosUI.api.init_topics();
        RosUI.api.init_services();
    },
    set_expression: function (faceStr, intensity) {
        RosUI.api.topics.expression.publish(
            new ROSLIB.Message({
                robotname: RosUI.api.config.robotname,
                expr: {
                    exprname: faceStr,
                    intensity: intensity
                }
            })
        );
    },
    point_head: function (angles) {
        $.extend({yaw: 0, pitch: 0, roll: 0}, angles);

        console.log('pointing head:');
        console.log(angles);

        RosUI.api.topics.pointHeadTopic.publish(
            new ROSLIB.Message(angles)
        );
    },
    expression_list: function (success) {
        RosUI.api.services.expression_list.callService(new ROSLIB.ServiceRequest({
                robotname: "dmitry"
            }), success
        );
    },
    init_topics: function () {
        //Publish topic - to be deleted
        RoboInterface.motorCmdTopic = new ROSLIB.Topic({
            ros: RosUI.api.ros,
            name: '/dmitry/pololu/cmd_pololu',
            messageType: 'ros_pololu_servo/servo_pololu'
        });

        RoboInterface.motortopicParams = {
            face: {
                ros: RosUI.api.ros,
                name: '/dmitry/dmitry_face/cmd_pololu',
                messageType: 'ros_pololu_servo/servo_pololu'
            },
            eyes: {
                ros: RosUI.api.ros,
                name: '/dmitry/dmitry_eyes/cmd_pololu',
                messageType: 'ros_pololu_servo/servo_pololu'
            },
            jaw: {
                ros: RosUI.api.ros,
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

        RosUI.api.topics = {
            cmdBlender: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/cmd_blendermode',
                messageType: 'std_msgs/String'
            }),
            cmdBllink: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/dmitry/cmd_blink',
                messageType: 'std_msgs/String'
            }),
            cmdTree: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/dmitry/behavior_switch',
                messageType: 'std_msgs/String'
            }),
            speech_topic: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/dmitry/chatbot_speech',
                messageType: 'chatbot/ChatMessage'
            }),
            expression: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/dmitry/make_coupled_face_expr',
                messageType: 'basic_head_api/MakeCoupledFaceExpr'
            }),
            pointHeadTopic: new ROSLIB.Topic({
                ros: RosUI.api.ros,
                name: '/dmitry/point_head',
                messageType: 'basic_head_api/PointHead'
            })
        }
    },
    init_services: function() {
        RosUI.api.services = {
            expression_list: new ROSLIB.Service({
                ros: RosUI.api.ros,
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
        var motorConf = RosUI.api.config.motors;

        for (var i = 0; i < motorConf.length; i++) {
            if (motorConf[i].name == name)
                return motorConf[i];
        }
    }
};

//Utility function. Builds a wrapper function around the given function and
//makes sure it is not invoked in shorter time intervals than 'millis'. The
//last call is guaranteed to make it after the timer goes out.
var limitCallRate = function (millis, func) {
    var timeout = null;
    var last_args = null;

    function fire() {
        timeout = null;
        if (last_args != null) {
            args = last_args;
            last_args = null;
            timeout = setTimeout(fire, millis);
            func.apply(null, args); //Apply the last saved arguments
        }
    }

    return function () {
        last_args = arguments;
        if (timeout == null) {
            fire();
        }
    };
}

//Main Class
var RoboInterface = {

    $: $({}), //Event proxy. Events are triggered and bound to this object.
    // Resets blender mode
    startBlenderMode: function startBlenderMode() {
//        var cmdBlender = new ROSLIB.Topic({
//            ros: ros,
//            name: '/cmd_blendermode',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'Dummy'
//        });
//        cmdBlender.publish(msg);
//        var cmdBllink = new ROSLIB.Topic({
//            ros: ros,
//            name: '/dmitry/cmd_blink',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'dmitry:stop'
//        });
//        cmdBllink.publish(msg);
//        var cmdTree = new ROSLIB.Topic({
//            ros: ros,
//            name: '/dmitry/behavior_switch',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'btree_off'
//        });
//        cmdTree.publish(msg);

    },

    //Keeps the incomming motor messages from flooding.
    sendMotorCmd: limitCallRate(30, function () {
        RoboInterface._sendMotorCmd.apply(RoboInterface, arguments);
    }),

    _sendMotorCmd: function (confEntry, angle, speed, acc) {
        if (!confEntry.topic || confEntry.topic == 'none')
            return false;
        var topicParams = RoboInterface.motortopicParams[confEntry.topic];
        var topic = RoboInterface.motorCmdTopics[confEntry.topic];
        if (topicParams.messageType == 'std_msgs/Float64') {
            var cmd = new ROSLIB.Message({data: Math.min(Math.max(angle, confEntry.min), confEntry.max)});
        } else {
            var cmd = new ROSLIB.Message({
                id: confEntry.motorid,
                angle: Math.min(Math.max(angle, confEntry.min), confEntry.max),
                speed: speed || confEntry.speed,
                acceleration: acc || confEntry.acceleration
            });

        }
        topic.publish(cmd);
        //RoboInterface.motorCmdTopic.publish(cmd);
    },

    sendDefaultMotorCmds: function () {
        for (var i = 0; i < this.motorConf.length; i++) {
            this._sendMotorCmd(this.motorConf[i], this.motorConf[i].default);
        }
        ;
        this.pointHead();
    },

    getValidFaceExprs: function (callback) {
        this.validFaceExprsClient.callService(
            new ROSLIB.ServiceRequest({
                robotname: "dmitry"
            }),
            callback
        );
    },

    makeFaceExpr: function (faceStr, intensity) {
        this.makeFaceExprTopic.publish(
            new ROSLIB.Message({
                robotname: "dmitry",
                expr: {
                    exprname: faceStr,
                    intensity: intensity
                }
            })
        );
    },

    //Keeps the incomming motor messages from flooding.
    pointHead: limitCallRate(30, function () {
        RoboInterface._pointHead.apply(RoboInterface, arguments);
    }),

    _pointHead: function (new_angles) {
        var angles = {yaw: 0, pitch: 0, roll: 0};

        this._pointHead = function (new_angles) {
            $.extend(angles, new_angles);
            RoboInterface.pointHeadTopic.publish(
                new ROSLIB.Message(angles)
            );
        };

        return this._pointHead(new_angles);
    },

    //Loads the config file and connects to ROS
    connect: function (address) {

        function connectROS() {
            //Connect to rosbridge
            ros = new ROSLIB.Ros({
                url: address
            }).on("connection", function (e) {
                RoboInterface.$.trigger("connection");
                RoboInterface.startBlenderMode();
                RoboInterface.sendDefaultMotorCmds();
            });

            ros.on('connection', function () {
                $('#app-connecting').hide();
                $('#app-pages').fadeIn();
            });

            ros.on('close', function () {
                $('#notifications .label').hide();
                $('#app-connection-error').show();
                $('#app-title').html('');
            });

            ros.on('error', function (error) {
                $('#notifications .label').hide();
                $('#app-connection-error').show();
                $('#app-title').html('');
            });

            //Publish topic - to be deleted
            RoboInterface.motorCmdTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/dmitry/pololu/cmd_pololu',
                messageType: 'ros_pololu_servo/servo_pololu'
            });
            RoboInterface.motortopicParams = {
                neck1: {
                    ros: ros,
                    name: '/fritz/neck1_controller/command',
                    messageType: 'std_msgs/Float64'
                },                
                neck2: {
                    ros: ros,
                    name: '/fritz/neck2_controller/command',
                    messageType: 'std_msgs/Float64'
                },
                neck3: {
                    ros: ros,
                    name: '/fritz/neck3_controller/command',
                    messageType: 'std_msgs/Float64'
                },
                neck4: {
                    ros: ros,
                    name: '/fritz/neck4_controller/command',
                    messageType: 'std_msgs/Float64'
                },
                neck5: {
                    ros: ros,
                    name: '/fritz/neck5_controller/command',
                    messageType: 'std_msgs/Float64'
                },
            };
            // Publish and subscribe to topics
            RoboInterface.motorCmdTopics = {}
            $.each(RoboInterface.motortopicParams, function (k, p) {
                RoboInterface.motorCmdTopics[k] = new ROSLIB.Topic(p);
                RoboInterface.motorCmdTopics[k].subscribe(function (msg) {

                    //  RoboInterface.$.trigger("onMotorCmd", {
                    //   msg: msg,
                    //    topic: p,
                    //    confEntry: getConfFromTopicID(msg,k)
                    //  });
                });
            });


            RoboInterface.makeFaceExprTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/dmitry/make_coupled_face_expr',
                messageType: 'basic_head_api/MakeCoupledFaceExpr'
            });
            RoboInterface.pointHeadTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/dmitry/point_head',
                messageType: 'basic_head_api/PointHead'
            });


            //Set up services
            RoboInterface.validFaceExprsClient = new ROSLIB.Service({
                ros: ros,
                name: '/dmitry/valid_coupled_face_exprs',
                serviceType: 'basic_head_api/ValidCoupledFaceExprs'
            });
        };
        this.$.on("configload", connectROS);

        $.ajax({
            url: "config.yaml",
            dataType: "text",
            success: function (data) {
                RoboInterface.motorConf = jsyaml.load(data);
                RoboInterface.$.trigger("configload");
            }
        });

        return this;
    }
};

//Utility function
var getConfFromTopicID = (function () {
    var motorID2Conf = {};

    RoboInterface.$.on("configload", function () {
        var motorConf = RoboInterface.motorConf;
        for (var i = 0; i < motorConf.length; i++) {
            var id = motorConf[i].motorid || "";
            motorID2Conf[motorConf[i].topic + id] = motorConf[i];
        }
    });

    return function (msg, topic) {
        var id = msg.id || "";
        return motorID2Conf[topic + id];
    };
})();

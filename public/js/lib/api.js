RosUI.api = {
    setExpression: function (name, intensity) {
        console.log('setting expression to: ' + name + ", intensity: " + intensity);

        RosUI.ros.topics.expression.publish(
            new ROSLIB.Message({
                robotname: RosUI.ros.config.robotname,
                expr: {
                    exprname: name,
                    intensity: intensity
                }
            })
        );
    },
    pointHead: function (angles) {
        if (typeof angles == 'undefined')
            angles = {};

        angles = $.extend({yaw: 0, pitch: 0, roll: 0}, angles);

        console.log('pointing head:');
        console.log(angles);

        RosUI.ros.topics.pointHeadTopic.publish(
            new ROSLIB.Message(angles)
        );
    },
    expressionList: function (success) {
        RosUI.ros.services.expressionList.callService(new ROSLIB.ServiceRequest({
                robotname: "dmitry"
            }), success
        );
    },
    playAnimation: function (animation) {
        RosUI.ros.topics.animations.publish(
            new ROSLIB.Message({
                data: 'play:' + animation
            })
        );
    },
    sendChatMessage: function(text) {
        var message = new ROSLIB.Message({
            utterance: text,
            confidence: Math.round(0.9 * 100)
        });

        RosUI.ros.topics.speech_topic.publish(message);
    },
    sendMotorCommand: RosUI.utilities.limitCallRate(30, function () {
        RosUI.api._sendMotorCommand.apply(RosUI.api, arguments);
    }),
    _sendMotorCommand: function (confEntry, angle, speed, acc) {
        if (!confEntry.topic || confEntry.topic == 'none')
            return false;
        var topicParams = RosUI.ros.topics[confEntry.topic];
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
        RosUI.ros.topics[confEntry.topic].publish(cmd);
    },
    setDefaultMotorValues: function () {
        for (var i = 0; i < RosUI.ros.config.motors.length; i++) {
            this._sendMotorCommand(RosUI.ros.config.motors[i], RosUI.ros.config.motors[i].default);
        }
    }
};

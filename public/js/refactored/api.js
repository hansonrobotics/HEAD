RosUI.api = {
    set_expression: function (name, intensity) {
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
    point_head: function (angles) {
        $.extend({yaw: 0, pitch: 0, roll: 0}, angles);

        console.log('pointing head:');
        console.log(angles);

        RosUI.ros.topics.pointHeadTopic.publish(
            new ROSLIB.Message(angles)
        );
    },
    expression_list: function (success) {
        RosUI.ros.services.expression_list.callService(new ROSLIB.ServiceRequest({
                robotname: "dmitry"
            }), success
        );
    },
    send_motor_command: RosUI.ros.limit_call_rate(30, function () {
        RosUI.api._send_motor_command.apply(RosUI.api, arguments);
    }),
    _send_motor_command: function (confEntry, angle, speed, acc) {
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
    set_default_motor_values: function () {
        for (var i = 0; i < RosUI.ros.config.motors.length; i++) {
            this._send_motor_command(RosUI.ros.config.motors[i], RosUI.ros.config.motors[i].default);
        }
    }
};

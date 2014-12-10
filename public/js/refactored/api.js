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
    }
};

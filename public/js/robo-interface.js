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
};

//Main Class
var RoboInterface = {

    $: $({}), //Event proxy. Events are triggered and bound to this object.

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
        for (var i = 0; i < RosUI.api.config.motors.length; i++) {
            this._sendMotorCmd(RosUI.api.config.motors[i], RosUI.api.config.motors[i].default);
        }
        //this.pointHead();
    }

    ////Keeps the incomming motor messages from flooding.
    //pointHead: limitCallRate(30, function () {
    //    RoboInterface._pointHead.apply(RoboInterface, arguments);
    //}),
    //
    //_pointHead: function (new_angles) {
    //    var angles = {yaw: 0, pitch: 0, roll: 0};
    //
    //    this._pointHead = function (new_angles) {
    //        $.extend(angles, new_angles);
    //        RoboInterface.pointHeadTopic.publish(
    //            new ROSLIB.Message(angles)
    //        );
    //    };
    //
    //    return this._pointHead(new_angles);
    //}
};
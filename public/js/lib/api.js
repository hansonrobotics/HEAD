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
    pointHead: RosUI.utilities.limitCallRate(30, function () {
        RosUI.api._pointHead.apply(RosUI.api, arguments);
    }),
    _pointHead: function (angles) {
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
                robotname: "arthur"
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
                joint_name: confEntry.name,
                position: Math.min(Math.max(angle, confEntry.min), confEntry.max),
                speed: (speed || confEntry.speed || 100)/255,
                acceleration: (acc || confEntry.acceleration || 50)/255
            });

        }
        RosUI.ros.topics[confEntry.topic].publish(cmd);
    },
    setDefaultMotorValues: function () {
        for (var i = 0; i < RosUI.ros.config.motors.length; i++) {
            this._sendMotorCommand(RosUI.ros.config.motors[i], RosUI.ros.config.motors[i].default);
        }
    },
    /**
     * Passes a list of available gestures to success function
     *
     * @param success
     */
    getAvailableGestures: function(success) {
        RosUI.ros.topics.available_gestures.subscribe(function (message) {
            success(message.data);
        });
    },
    /**
     * Set a gesture
     *
     * @param name
     * @param repeat
     * @param speed
     * @param magnitude
     */
    setGesture: function (name, repeat, speed, magnitude) {
        if (typeof repeat == 'undefined')
            repeat = 1;

        if (typeof speed == 'undefined')
            speed = 0.5;

        if (typeof magnitude == 'undefined')
            magnitude = 0.5;

        RosUI.ros.topics.set_gesture.publish(
            new ROSLIB.Message({
                name: name,
                repeat: repeat,
                speed: speed,
                magnitude: magnitude
            })
        );
    },
    /**
     * Passes a list of available emotions to success function
     *
     * @param success
     */
    getAvailableEmotionStates: function (success) {
        RosUI.ros.topics.available_emotion_states.subscribe(function (message) {
            success(message.data);
        });
    },
    /**
     * Set an emotion, call multiple times to blend emotions together
     *
     * @param name
     * @param magnitude 0..1
     * @param duration array [seconds, nanoseconds]
     */
    setEmotion: function(name, magnitude, duration) {
        if (typeof magnitude == 'undefined')
            magnitude = 0.5;

        if (typeof duration == 'undefined')
            duration = {secs: 1, nsecs: 0};

        RosUI.ros.topics.set_emotion_state.publish(
            new ROSLIB.Message({
                name: name,
                magnitude: magnitude,
                duration: duration
            })
        );
    },
    blenderMode: {
        enable: function() {
            RosUI.ros.services.headPauMux.callService(new ROSLIB.ServiceRequest("/blender_api/get_pau"));
            RosUI.ros.services.neckPauMux.callService(new ROSLIB.ServiceRequest("/blender_api/get_pau"));
        },
        disable: function() {
            RosUI.ros.services.headPauMux.callService(new ROSLIB.ServiceRequest("/fritz/no_pau"));
            RosUI.ros.services.neckPauMux.callService(new ROSLIB.ServiceRequest("/fritz/cmd_neck_pau"));
        }
    }
};

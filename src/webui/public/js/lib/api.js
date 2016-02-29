define(['jquery', 'roslib', './utilities'], function ($, ROSLIB, utilities) {
    var api = {
        config: {},
        setExpression: function (name, intensity) {
            console.log('setting expression to: ' + name + ", intensity: " + intensity);

            api.topics.expression.publish(
                new ROSLIB.Message({
                    exprname: name,
                    intensity: parseFloat(intensity)
                })
            );
        },
        pointHead: utilities.limitCallRate(30, function () {
            api._pointHead.apply(api, arguments);
        }),
        _pointHead: function (angles) {
            if (typeof angles == 'undefined')
                angles = {};

            angles = $.extend({yaw: 0, pitch: 0, roll: 0}, angles);

            console.log('pointing head:');
            console.log(angles);

            api.topics.pointHeadTopic.publish(
                new ROSLIB.Message(angles)
            );
        },
        expressionList: function (success) {
            api.services.expressionList.callService(new ROSLIB.ServiceRequest(), success);
        },
        setExpressionsParam: function (expressions) {
            var param = new ROSLIB.Param({
                ros: api.ros,
                name: '/' + api.config.robot + '/expressions'
            });
            param.set(expressions);
        },
        getAnimations: function (callback) {
            var param = new ROSLIB.Param({
                ros: api.ros,
                name: '/' + api.config.robot + '/animations'
            });

            param.get(function (animations) {
                callback(animations);
            });
        },
        setAnimations: function (animations) {
            var param = new ROSLIB.Param({
                ros: api.ros,
                name: '/' + api.config.robot + '/animations'
            });
            param.set(animations);
        },
        updateAnimations: function (animations, successCallback, errorCallback) {
            var self = this;
            $.ajax('/animations/update/' + api.config.robot, {
                dataType: 'json',
                data: JSON.stringify({animations: animations}),
                method: 'POST',
                success: function (response) {
                    self.setAnimations(animations);

                    if (typeof successCallback == 'function')
                        successCallback(response);
                },
                error: function (xhr, status, error) {
                    if (typeof errorCallback == 'function')
                        errorCallback(xhr, status, error);
                }
            });
        },
        playAnimation: function (animation, fps) {
            api.topics.play_animation.publish(
                new ROSLIB.Message({
                    animation: animation,
                    fps: fps
                })
            );
        },
        sendChatMessage: function (text) {
            console.log('Sending message: ' + text);

            var message = new ROSLIB.Message({
                utterance: text,
                confidence: Math.round(0.9 * 100)
            });

            api.topics.speech_topic.publish(message);
        },
        sendMotorCommand: utilities.limitCallRate(30, function () {
            api._sendMotorCommand.apply(api, arguments);
        }),
        _sendMotorCommand: function (confEntry, angle, speed, acc) {
            console.log('Sending motor command:');
            console.log([confEntry, angle, speed, acc]);

            var topicParams = api.topics[confEntry.topic],
                cmd;

            if (!confEntry.topic || confEntry.topic == 'none' || typeof topicParams == 'undefined')
                return false;

            if (topicParams.messageType == 'std_msgs/Float64') {
                cmd = new ROSLIB.Message({data: Math.min(Math.max(angle, confEntry.min), confEntry.max)});
            } else {
                cmd = new ROSLIB.Message({
                    joint_name: confEntry.name.toString(),
                    position: Math.min(Math.max(angle, confEntry.min), confEntry.max),
                    speed: 0.05,
                    acceleration: (acc || confEntry.acceleration || 50) / 255
                });
            }

            api.topics[confEntry.topic].publish(cmd);
        },
        getPololuMotorTopics: function (success) {
            api.services.topicsForType.callService({type: 'ros_pololu/MotorCommand'}, function (response) {
                success(response.topics);
            }, function (error) {
                console.log(error);
            });
        },
        /**
         * Passes a list of available soma states to success function
         *
         * @param success
         */
        getAvailableSomaStates: function (success) {
            api.topics.available_soma_states.unsubscribe();
            api.topics.available_soma_states.removeAllListeners();
            api.topics.available_soma_states.subscribe(function (message) {
                success(message.data);
            });
        },
        /**
         * Set a soma state
         *
         * @param name
         * @param rate
         * @param magnitude
         * @param ease_in
         */
        setSomaState: function (name, rate, magnitude, ease_in) {
            if (typeof rate == 'undefined')
                rate = 1.0;

            if (typeof magnitude == 'undefined')
                magnitude = 1.0;

            if (typeof ease_in == 'undefined')
                ease_in = 0.5;

            api.topics.set_soma_state.publish(
                new ROSLIB.Message({
                    name: name,
                    rate: rate,
                    magnitude: magnitude,
                    ease_in: ease_in
                })
            );
        },
        /**
         * Passes a list of available gestures to success function
         *
         * @param success
         */
        getAvailableGestures: function (success) {
            api.topics.available_gestures.unsubscribe();
            api.topics.available_gestures.removeAllListeners();
            api.topics.available_gestures.subscribe(function (message) {
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
                magnitude = 1;

            api.topics.set_gesture.publish(
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
            api.topics.available_emotion_states.unsubscribe();
            api.topics.available_emotion_states.removeAllListeners();
            api.topics.available_emotion_states.subscribe(function (message) {
                success(message.data);
            });
        },
        /**
         * Set an emotion, call multiple times to blend emotions together
         *
         * @param name
         * @param magnitude 0..1
         * @param duration mixed array or number of seconds [seconds, nanoseconds]
         */
        setEmotion: function (name, magnitude, duration) {
            if (typeof magnitude == 'undefined')
                magnitude = 1;

            if ($.isNumeric(duration))
                duration = {
                    secs: Math.floor(duration),
                    nsecs: Math.floor((duration - Math.floor(duration)) * 1000)
                };
            else if (typeof duration != 'object')
                duration = {secs: 1, nsecs: 0};

            api.topics.set_emotion_state.publish(
                new ROSLIB.Message({
                    name: name,
                    magnitude: magnitude,
                    duration: duration
                })
            );
        },
        setFaceTarget: function (x, y, z) {
            api.topics.set_face_target.publish(
                new ROSLIB.Message({
                    x: x,
                    y: y,
                    z: z
                })
            );
        },
        setGazeTarget: function (x, y, z) {
            api.topics.set_gaze_target.publish(
                new ROSLIB.Message({
                    x: x,
                    y: y,
                    z: z
                })
            );
        },
        webSpeech: function (text, lang) {
            var topic;
            if (!lang) {
                topic = api.topics.web_responses['default'];
            } else {
                topic = api.topics.web_responses[lang];
            }
            topic.publish(
                new ROSLIB.Message({
                    data: text
                })
            );
        },
        robotSpeech: function (text, lang) {
            var topic;
            if (!lang) {
                topic = api.topics.tts['default'];
            } else {
                topic = api.topics.tts[lang];
            }
            topic.publish(
                new ROSLIB.Message({
                    data: text
                })
            );
        },
        blenderMode: {
            enable: function () {
                api.services.headPauMux.callService(new ROSLIB.ServiceRequest({topic: "/blender_api/get_pau"}), function () {
                    console.log('face enabled');
                    return 0;
                });
                api.services.neckPauMux.callService(new ROSLIB.ServiceRequest({topic: "/blender_api/get_pau"}), function () {
                    return 0;
                });
            },
            disable: function () {
                api.services.headPauMux.callService(new ROSLIB.ServiceRequest({topic: "/" + api.config.robot + "/no_pau"}), function () {
                    return 0;
                });
                api.services.neckPauMux.callService(new ROSLIB.ServiceRequest({topic: "/" + api.config.robot + "/cmd_neck_pau"}), function () {
                    return 0;
                });
            },
            disableFace: function () {
                api.services.headPauMux.callService(new ROSLIB.ServiceRequest({topic: "/" + api.config.robot + "/no_pau"}), function () {
                    console.log('face disabled');
                    return 0;
                });
            }
        },
        getMotorsFromParam: function (callback) {
            var param = new ROSLIB.Param({ros: api.ros, name: '/' + api.config.robot + '/motors'}),
                self = this;

            param.get(function (motors) {
                motors.sort(function (a, b) {
                    return parseFloat(a.sort_no) - parseFloat(b.sort_no)
                });
                self.createMotorTopics(motors);
                callback(motors);
            });
        },
        createMotorTopics: function (motors) {
            for (var i = 0; motors && i < motors.length; i++) {
                // Create topics for specific motors
                if (!(motors[i]['topic'] in api.topics)) {
                    if ('motor_id' in motors[i]) {
                        // Pololu
                        api.topics[motors[i]['topic']] = new ROSLIB.Topic({
                            ros: api.ros,
                            name: '/' + api.config.robot + '/' + motors[i]['topic'] + '/command',
                            messageType: 'ros_pololu/MotorCommand',
                            throttle_rate: 5
                        });
                    } else {
                        // Dynamixel
                        api.topics[motors[i]['topic']] = new ROSLIB.Topic({
                            ros: api.ros,
                            name: '/' + api.config.robot + '/' + motors[i]['topic'] + '_controller/command',
                            messageType: 'std_msgs/Float64'
                        });
                    }
                }
            }
        },
        getRobotName: function (callback) {
            var param = new ROSLIB.Param({ros: api.ros, name: '/robot_name'});
            param.get(callback);
        },
        getRobotLang: function (success) {
            var param = new ROSLIB.Param({ros: api.ros, name: '/' + api.config.robot + '/lang'});

            param.get(function (lang) {
                if (typeof success == 'function') success(lang);
            });
        },
        setRobotLang: function (lang) {
            var language = null,
                param = new ROSLIB.Param({ros: api.ros, name: '/' + api.config.robot + '/lang'});
            param.set(lang);

            if (lang == 'en') language = 'en_us';
            else if (lang == 'zh') language = 'zh_ch';

            if (language) this.setDynParam('/' + this.config.robot + '/speech_recognizer', 'language', language);
        },
        /**
         * Passes a "|" separated string of available scripts
         *
         * @param success
         */
        getAvailableScripts: function (success) {
            var callback = function (message) {
                api.topics.scripts_available.unsubscribe(callback);
                success(message.data.split("|"));
            };
            api.topics.scripts_available.subscribe(callback);
        },
        /**
         * Executes given script
         *
         * @param scripts
         */
        executeScript: function (script) {
            var cmd = new ROSLIB.Message({data: script})
            api.topics.execute_scripts.publish(cmd)
        },
        getTtsLength: function (text, lang, success) {
            api.services.tts_length.callService(new ROSLIB.ServiceRequest({
                txt: text,
                lang: lang
            }), success, function (error) {
                console.log(error);
            });
        },
        getAnimationLength: function (animation, success) {
            api.services.get_animation_length.callService(new ROSLIB.ServiceRequest({animation: animation}), success, function (error) {
                console.log(error);
            });
        },
        enableInteractionMode: function () {
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_on'}));
        },
        disableInteractionMode: function () {
            api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_off'}));
        },
        set_look_at_face: function (f_id) {
            api.topics.set_look_at_face.publish(new ROSLIB.Message({face_event: 'track_face', face_id: f_id}));
        },
        setDynParam: function (node, name, value, options) {
            if (typeof options != 'object') options = {};

            var data = {},
                type = null,
                service = new ROSLIB.Service({
                    ros: this.ros,
                    name: node + '/set_parameters',
                    messageType: 'dynamic_reconfigure/Reconfigure'
                });

            switch (typeof value) {
                case 'boolean':
                    type = 'bools';
                    break;
                case 'number':
                    if (value % 1 === 0)
                        type = 'ints';
                    else
                        type = 'doubles';
                    break;
                case 'string':
                    type = 'strs';
                    break;
                case 'object':
                    type = 'groups';

            }

            data[type] = [{}];
            data[type][0]['name'] = name;
            data[type][0]['value'] = value;

            service.callService(new ROSLIB.ServiceRequest({config: data}), function () {
                if (typeof options.success == 'function') options.success();
            }, function (error) {
                if (typeof options.error == 'function') options.error(error);
            });
        },
        setRosParam: function (name, value) {
            var param = new ROSLIB.Param({
                ros: api.ros,
                name: name
            });
            param.set(value);
        },
        getRosParam: function (name, success) {
            var param = new ROSLIB.Param({
                ros: api.ros,
                name: name
            });

            param.get(success);
        },
        /**
         * Subscribes for performance event topics
         *
         * @param success
         */
        subPerformanceEvents: function (success) {
            api.topics.performance_events.unsubscribe();
            api.topics.performance_events.removeAllListeners();
            api.topics.performance_events.subscribe(function (message) {
                success(message);
            });
        },
        enableTtsOperatorMode: function () {
            this.setTtsMux('web_responses');
        },
        disableTtsOperatorMode: function () {
            this.setTtsMux('chatbot_responses');
        },
        setTtsMux: function (topic) {
            api.services.tts_select.default.callService(new ROSLIB.ServiceRequest({topic: topic}));
            api.services.tts_select.en.callService(new ROSLIB.ServiceRequest({topic: topic + '_en'}));
            api.services.tts_select.zh.callService(new ROSLIB.ServiceRequest({topic: topic + '_zh'}));
        }
    };

    return api;
});

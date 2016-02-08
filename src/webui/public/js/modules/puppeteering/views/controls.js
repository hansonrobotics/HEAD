define(['marionette', 'tpl!./templates/controls.tpl', 'lib/api', 'roslib', 'underscore', 'jquery'],
    function (Marionette, template, api, ROSLIB, _, $) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                modeButtons: '.app-animation-mode-button',
                animationContainer: '.app-animation-buttons',
                poseContainer: '.app-pose-buttons'
            },
            events: {
                'click @ui.modeButtons': 'changePpMode'
            },
            onRender: function () {
                var self = this;
                api.getAvailableGestures(function (gestures) {
                    _.each(gestures, function (gesture) {
                        self.ui.animationContainer.append($('<button>').attr({
                            type: 'button',
                            'class': 'btn btn-default',
                            'data-name': gesture
                        }).click(function () {
                            api.setGesture(gesture);
                        }).html(gesture)).append(' ');
                    });
                });

                api.getAvailableEmotionStates(function (emotions) {
                    _.each(emotions, function (emotion) {
                        self.ui.poseContainer.append($('<button>').attr({
                            type: 'button',
                            'class': 'btn btn-default',
                            'data-name': emotion
                        }).click(function () {
                            api.setEmotion(emotion);
                        }).html(emotion)).append(' ');
                    });
                });
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            }
        });
    });

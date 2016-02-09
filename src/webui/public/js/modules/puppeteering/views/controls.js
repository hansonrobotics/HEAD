define(['marionette', 'tpl!./templates/controls.tpl', 'lib/api', 'roslib', 'underscore', 'jquery', 'lib/crosshair-slider'],
    function (Marionette, template, api, ROSLIB, _, $) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                modeButtons: '.app-animation-mode-button',
                animationContainer: '.app-animation-buttons',
                poseContainer: '.app-pose-buttons',
                headControl: '.app-head-control',
                eyeControl: '.app-eye-control',
                speechInput: '.app-speech-input',
                sayButton: '.app-say-button'
            },
            events: {
                'click @ui.modeButtons': 'changePpMode',
                'keyup @ui.speechInput': 'speechInputKeyPress',
                'click @ui.sayButton': 'saySpeech'
            },
            saySpeech: function () {
                var speech = this.ui.speechInput.val();
                if (speech) {
                    var lang = this.options.interactionView ? this.options.interactionView.language : undefined;
                    this.ui.speechInput.val('');
                    api.robotSpeech(speech, lang);
                }
            },
            speechInputKeyPress: function (e) {
                if (e.keyCode == 13) this.saySpeech();
            },
            onRender: function () {
                this.showAnimationButtons();
                this.showPoseButtons();
                this.buildHeadCrosshair();
                this.buildEyeCrosshair();
            },
            buildHeadCrosshair: function () {
                this.ui.headControl.crosshairsl({
                    change: function (e, ui) {
                        api.setFaceTarget(1, ui.xval / 50, ui.yval / 50);
                    }
                });
            },
            buildEyeCrosshair: function () {
                this.ui.eyeControl.crosshairsl({
                    change: function (e, ui) {
                        api.setGazeTarget(1, ui.xval / 50, ui.yval / 50);
                    }
                });
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            },
            showAnimationButtons: function () {
                var self = this;
                api.getAvailableGestures(function (animation) {
                    _.each(animation, function (gesture) {
                        self.ui.animationContainer.append($('<button>').attr({
                            type: 'button',
                            'class': 'btn btn-default',
                            'data-name': gesture
                        }).click(function () {
                            api.setGesture(gesture);
                        }).html(gesture)).append(' ');
                    });
                });
            },
            showPoseButtons: function () {
                var self = this;
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
            }
        });
    });

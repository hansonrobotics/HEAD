define(['marionette', './templates/animation_mode.tpl', 'lib/api', 'jquery', 'roslib', 'annyang'],
    function (Marionette, template, api, $, ROSLIB, annyang) {
        return Marionette.ItemView.extend({
            ui: {
                modeButtons: '.app-gesture-pp',
                btOnButton: ".app-gesture-bt-on",
                btOffButton: ".app-gesture-bt-off",
                webspeechOnButton: ".app-webspeech-on",
                webspeechOffButton: ".app-webspeech-off",
                btFTButton: ".app-gesture-bt-ft",
                lsOnButton: ".app-gesture-ls-on",
                lsOffButton: ".app-gesture-ls-off",
                btFTOffButton: ".app-gesture-bt-ft-off",
                behaviorContainer: '.app-behavior-container',
                lipsyncContainer: '.app-lipsync-container',
                modeContainer: '.app-mode-container',
                webspeechContainer: '.app-webspeech-container'
            },
            template: template,
            events: {
                'click @ui.btOnButton': "btOn",
                'click @ui.btOffButton': "btOff",
                'click @ui.webspeechOnButton': "webspeechOn",
                'click @ui.webspeechOffButton': "webspeechOff",
                'click @ui.btFTButton': "btFT",
                'click @ui.btFTOffButton': "btFTOff",
                'click @ui.modeButtons': "changePpMode",
                'click @ui.lsOnButton': "lsOn",
                'click @ui.lsOffButton': "lsOff"
            },
            initialize: function (options) {
                this.mergeOptions(options, ['settings', 'language']);
                if (!this.settings || !this.settings.constructor === Array)
                    this.settings = ['behavior', 'mode', 'lipsync'];
            },
            onAttach: function () {
                var self = this,
                    cssClass = 'col-md-' + 12 / this.settings.length;

                this.ui.behaviorContainer.addClass(cssClass);
                this.ui.modeContainer.addClass(cssClass);
                this.ui.lipsyncContainer.addClass(cssClass);

                if (!_.includes(this.settings, 'behavior'))
                    this.ui.behaviorContainer.hide();

                if (!_.includes(this.settings, 'mode'))
                    this.ui.modeContainer.hide();

                if (!_.includes(this.settings, 'lipsync'))
                    this.ui.lipsyncContainer.hide();

                if (!_.includes(this.settings, 'webspeech')) {
                    this.ui.webspeechContainer.hide();
                } else {
                    var speechActiveCallback = function (msg) {
                        if (self.isDestroyed)
                            api.topics.speech_active.unsubscribe(speechActiveCallback);
                        else
                            self.speechActiveCallback(msg);
                    };
                    api.topics.speech_active.subscribe(speechActiveCallback);
                }
            },
            btOn: function () {
                api.enableInteractionMode();
                api.setBTMode(api.btModes.C_ALL);
            },
            btOff: function () {
                api.disableInteractionMode();
            },
            lsOn: function () {
                api.setDynParam('/' + api.config.robot + '/anno_lipsync', 'lipsync', true)
            },
            lsOff: function () {
                api.setDynParam('/' + api.config.robot + '/anno_lipsync', 'lipsync', false)
            },
            btFT: function () {
                api.setBTMode(api.btModes.C_FACE | api.btModes.C_EYES);
                api.enableInteractionMode();
            },
            btFTOff: function () {
                api.setBTMode(api.btModes.C_ALL - (api.btModes.C_FACE | api.btModes.C_EYES));
                api.enableInteractionMode();
            },
            changePpMode: function (e) {
                var mode = $(e.target).data("mode") || 0;
                api.topics.set_animation_mode.publish(new ROSLIB.Message({data: mode}));
            },
            webspeechOn: function () {
                this.speechEnabled = true;
                this.ui.webspeechOnButton.addClass('active');
                this.ui.webspeechOffButton.removeClass('active');

                var self = this;
                annyang.abort();
                annyang.removeCommands();
                annyang.removeCallback();
                annyang.setLanguage(this.language == 'zh' ? 'zh-CN' : 'en-US');
                annyang.addCallback('start', function () {
                    console.log('starting speech recognition');
                    api.topics.chat_events.publish(new ROSLIB.Message({data: 'start'}));
                    api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechstart'}));
                });
                annyang.addCallback('end', function () {
                    console.log('end of speech');
                    api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechend'}));
                    api.topics.chat_events.publish(new ROSLIB.Message({data: 'end'}));
                });

                annyang.addCallback('error', function (error) {
                    console.log('speech recognition error');
                    console.log(error);
                });
                annyang.addCallback('result', function (results) {
                    if (results.length) {
                        api.sendChatMessage(results[0]);
                        api.loginfo('speech recognised: ' + results[0]);
                    }
                });

                annyang.start({
                    autoRestart: true,
                    continuous: true,
                    paused: false
                });
            },
            webspeechOff: function (silent) {
                if (!silent) {
                    this.ui.webspeechOnButton.removeClass('active');
                    this.ui.webspeechOffButton.addClass('active');
                    this.speechEnabled = false;
                }

                annyang.abort();
            },
            speechActiveCallback: function (msg) {
                if (this.speechEnabled) {
                    if (msg.data == 'start') {
                        this.speechPaused = true;
                        this.webspeechOff(true);
                    }
                } else if ((msg.data == 'stop') && this.speechPaused) {
                    this.webspeechOn();
                }
            }
        });
    });

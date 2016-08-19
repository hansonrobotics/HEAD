define(['application', 'marionette', 'tpl!./templates/node_settings.tpl', '../entities/node', 'lib/api', 'underscore',
        'jquery', 'jquery-ui', 'lib/crosshair-slider', 'select2'],
    function (App, Marionette, template, Node, api, _, $) {
        return Marionette.ItemView.extend({
            template: template,
            ui: {
                nodeProperties: '[data-node-property]',
                magnitudeSlider: '.app-magnitide-slider',
                startTime: '.app-node-start-time',
                duration: '.app-node-duration',
                crosshair: '.app-crosshair',
                speedSlider: '.app-speed-slider',
                speedLabel: '.app-speed-label',
                intervalSlider: '.app-interval-slider',
                intervalLabel: '.app-interval-label',
                magnitudeLabel: '.app-magnitude-label',
                fpsSlider: '.app-fps-slider',
                pitchLabel: '.app-pitch-label',
                pitchSlider: '.app-pitch-slider',
                volumeLabel: '.app-volume-label',
                volumeSlider: '.app-volume-slider',
                fpsLabel: '.app-fps-label',
                messageInput: '.app-node-message-input',
                kfModeSelect: 'select.app-blender-mode-select',
                btreeModeSelect: 'select.app-btree-mode-select',
                createButton: '.app-create-button',
                timeout: '.app-node-timeout',
                dialogTurnsInput: '.app-dialog-turns-input',
                noMatchInput: '.app-no-match-input',
                noSpeechInput: '.app-no-speech-input',
                chatTimeoutModeSelect: '.app-chat-timeout-mode-select',
                chatbotSelect: '.app-chatbot-select'
            },
            events: {
                'change @ui.duration': 'setDuration',
                'change @ui.startTime': 'setStartTime',
                'change @ui.messageInput': 'setMessage',
                'change @ui.btreeModeSelect': 'setBtreeMode',
                'change @ui.kfModeSelect': 'setBlenderMode',
                'click @ui.createButton': 'addPerformance',
                'change @ui.timeout': 'updateTimeout',
                'change @ui.dialogTurnsInput': 'updateDialogTurns',
                'change @ui.noMatchInput': 'updateNoMatch',
                'change @ui.noSpeechInput': 'updateNoSpeech',
                'change @ui.chatbotSelect': 'updateChatbot',
                'change @ui.chatTimeoutModeSelect': 'updateChatTimeoutMode'
            },
            modelEvents: {
                change: 'modelChanged'
            },
            modelChanged: function () {
                this.ui.startTime.val(this.model.get('start_time'));
                this.ui.duration.val(this.model.get('duration'));
                this.ui.timeout.val(this.model.get('timeout'));
                this.ui.dialogTurnsInput.val(this.model.get('dialog_turns'));
                this.ui.noMatchInput.val(this.model.get('no_match'));
                this.ui.noSpeechInput.val(this.model.get('no_speech'));

                if (this.model.hasProperty('blender_mode')) this.ui.kfModeSelect.val(this.model.get('blender_mode'));
                if (this.model.hasProperty('message')) this.ui.messageInput.val(this.model.get('message'));
                if (this.model.hasProperty('attention_region')) this.selectAttentionRegion();

                if (this.collection.contains(this.model)) {
                    this.ui.createButton.hide();
                } else {
                    this.ui.createButton.fadeIn();
                }
            },
            onAttach: function () {
                var self = this;

                this.properties = this.model.getConfig().properties;

                this.ui.nodeProperties.hide();
                if (this.properties)
                    _.each(this.properties, function (prop) {
                        self.ui.nodeProperties.filter('[data-node-property="' + prop + '"]').show();
                    });

                this.modelChanged();

                if (this.model.hasProperty('chat_timeout_mode'))
                    this.ui.chatTimeoutModeSelect.val(this.model.get('timeout_mode')).select2();

                if (this.model.hasProperty('chatbot')) {
                    this.ui.chatbotSelect.select2();
                    api.services.chatbot.bot_names.callService({}, function (data) {
                        if (data.success) {
                            _.each(JSON.parse(data.response)['response'], function (name) {
                                self.ui.chatbotSelect.append($('<option>').prop({value: name}).html(name));
                            });

                            self.ui.chatbotSelect.val(self.model.get('bot_name'))
                        }
                    })
                }

                if (this.model.hasProperty('speed')) {
                    this.ui.speedLabel.html(this.model.get('speed'));
                    this.ui.speedSlider.slider({
                        range: 'min',
                        animate: true,
                        min: 50,
                        max: 200,
                        value: this.model.get('speed') * 100,
                        slide: function (e, ui) {
                            var speed = ui.value / 100;
                            self.model.set('speed', speed);
                            self.ui.speedLabel.html(speed.toFixed(2));
                        }
                    });
                }

                if (this.model.hasProperty('interval')) {
                    this.ui.intervalLabel.html(this.model.get('interval'));
                    this.ui.intervalSlider.slider({
                        range: 'min',
                        animate: true,
                        min: 0.5,
                        max: 10,
                        value: this.model.get('interval'),
                        slide: function (e, ui) {
                            self.model.set('interval', ui.value);
                            self.ui.intervalLabel.html(ui.value.toFixed(2) + 's');
                        }
                    });
                }

                if (this.model.hasProperty('pitch')) {
                    this.ui.pitchLabel.html(this.model.get('pitch'));
                    this.ui.pitchSlider.slider({
                        range: 'min',
                        animate: true,
                        min: 0,
                        max: 150,
                        value: this.model.get('pitch') * 100,
                        slide: function (e, ui) {
                            var pitch = ui.value / 100;
                            self.model.set('pitch', pitch);
                            self.ui.pitchLabel.html(pitch.toFixed(2));
                        }
                    });
                }

                if (this.model.hasProperty('volume')) {
                    this.ui.volumeLabel.html(this.model.get('volume'));
                    this.ui.volumeSlider.slider({
                        range: 'min',
                        animate: true,
                        min: 0,
                        max: 150,
                        value: this.model.get('volume') * 100,
                        slide: function (e, ui) {
                            var volume = ui.value / 100;
                            self.model.set('volume', volume);
                            self.ui.volumeLabel.html(volume.toFixed(2));
                        }
                    });
                }

                if (this.model.hasProperty('magnitude')) {
                    var magnitude = this.model.get('magnitude');

                    if (magnitude instanceof Array && magnitude.length == 2)
                        magnitude = [magnitude[0] * 100, magnitude[1] * 100];
                    else // computable with previous version of single value magnitude
                        magnitude = [magnitude * 100, magnitude * 100];

                    this.ui.magnitudeLabel.html(magnitude[0] + '-' + magnitude[1] + '%');

                    // init slider
                    this.ui.magnitudeSlider.slider({
                        animate: true,
                        range: true,
                        values: magnitude,
                        slide: function (event, ui) {
                            self.model.set('magnitude', [ui.values[0] / 100, ui.values[1] / 100]);
                            self.ui.magnitudeLabel.html(ui.values[0] + '-' + ui.values[1] + '%');
                        }
                    });
                }

                if (this.model.hasProperty('blender_mode'))
                    $(this.ui.kfModeSelect).select2();

                if (this.model.hasProperty('fps')) {
                    this.ui.fpsLabel.html(Math.floor(self.model.get('fps')) + ' fps');
                    this.ui.fpsSlider.slider({
                        animate: true,
                        range: 'min',
                        min: 12,
                        max: 48,
                        value: this.model.get('fps'),
                        slide: function (e, ui) {
                            self.model.set('fps', ui.value);
                            self.ui.fpsLabel.html(Math.floor(self.model.get('fps')) + ' fps');
                        }
                    });
                }

                if (this.model.hasProperty('attention_region'))
                    this.buildCrosshair();

            },
            addPerformance: function () {
                this.collection.add(this.model);
            },
            setDuration: function () {
                this.model.set('duration', Number($(this.ui.duration).val()));
            },
            setMessage: function () {
                this.model.set('message', this.ui.messageInput.val());
            },
            setStartTime: function () {
                this.model.set('start_time', Number($(this.ui.startTime).val()));
            },
            setBlenderMode: function () {
                this.model.set('blender_mode', $(this.ui.kfModeSelect).val());
            },
            setBtreeMode: function () {
                this.model.set('mode', parseInt(this.ui.btreeModeSelect.val()));
            },
            buildCrosshair: function () {
                var self = this;
                $(this.ui.crosshair).crosshairsl({
                    xmin: -1,
                    xmax: 1,
                    xval: this.model.get('y') ? this.model.get('y') : 0,
                    ymin: -1,
                    ymax: 1,
                    yval: this.model.get('z') ? -1 * this.model.get('z') : 0,
                    change: function (e, ui) {
                        self.model.set('x', 1);
                        self.model.set('y', ui.xval);
                        self.model.set('z', -1 * ui.yval);

                        self.model.call();
                    }
                });

                self.model.set('x', 1);
                self.model.set('y', 0);
                self.model.set('z', 0);
            },
            selectAttentionRegion: function () {
                if (this.model.get('attention_region') == 'custom')
                    this.ui.crosshair.fadeIn();
                else
                    this.ui.crosshair.fadeOut();
            },
            updateTimeout: function () {
                this.model.set('timeout', this.ui.timeout.val());
            },
            updateDialogTurns: function () {
                this.model.set('dialog_turns', this.ui.dialogTurnsInput.val());
            },
            updateNoMatch: function () {
                this.model.set('no_match', this.ui.noMatchInput.val());
            },
            updateNoSpeech: function () {
                this.model.set('no_speech', this.ui.noSpeechInput.val());
            },
            updateChatbot: function () {
                this.model.set('bot_name', this.ui.chatbotSelect.val());
            },
            updateChatTimeoutMode: function () {
                this.model.set('timeout_mode', this.ui.chatTimeoutModeSelect.val());
            }
        });
    });

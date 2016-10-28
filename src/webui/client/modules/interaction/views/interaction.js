define(['application', "marionette", './message', "./templates/interaction.tpl", 'lib/api', '../entities/message_collection',
        'jquery', './faces', 'underscore', 'lib/speech_recognition', 'annyang', 'roslib', 'scrollbar', 'scrollbar-css', '../css/interaction'],
    function (app, Marionette, MessageView, template, api, MessageCollection, $, FacesView, _, speechRecognition, annyang, ROSLIB) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: MessageView,
            childViewContainer: '.app-messages',
            ui: {
                messages: '.app-messages',
                recordButton: '.app-record-button',
                messageInput: '.app-message-input',
                sendButton: '.app-send-button',
                shutUpButton: '.app-shut-up-button',
                unsupported: '.app-unsupported',
                supported: '.app-supported',
                recordContainer: '.record-container',
                footer: '.app-interaction-footer',
                languageButton: '.app-language-select button',
                recognitionSelectContainer: '.app-recognition-select',
                recognitionMethodButton: '.app-recognition-select button',
                adjustNoiseButton: '.app-adjust-noise-button',
                noiseContainer: '.app-noise-container',
                noiseSlider: '.app-noise-slider',
                noiseValue: '.app-noise-value .value',
                scrollbar: '.app-scrollbar',
                facesContainer: '.app-faces-container',
                rateGoodButton: '.app-rate-good-button',
                rateBadButton: '.app-rate-bad-button'
            },
            events: {
                'touchstart @ui.recordButton': 'toggleSpeech',
                'touchend @ui.recordButton': 'toggleSpeech',
                'click @ui.recordButton': 'toggleSpeech',
                'keypress @ui.messageInput': 'messageKeyUp',
                'click @ui.sendButton': 'sendClicked',
                'click @ui.shutUpButton': 'shutUpClicked',
                'click @ui.languageButton': 'languageButtonClick',
                'click @ui.recognitionMethodButton': 'recognitionButtonClick',
                'click @ui.adjustNoiseButton': 'adjustButtonClick',
                'click @ui.rateGoodButton': 'rateGoodClicked',
                'click @ui.rateBadButton': 'rateBadClicked',
            },
            childViewOptions: function () {
                return {
                    collection: this.collection,
                    interactionView: this
                };
            },
            initialize: function (options) {
                if (!options.collection)
                    this.collection = new MessageCollection();
            },
            onDestroy: function () {
                this.disableSpeech();
            },
            onRender: function () {
                var self = this;
                api.enableInteractionMode();
                this.addListeners();

                if (this.options.recognition_method)
                    this.setRecognitionMethod(this.options.recognition_method);

                if (this.options.hide_method_select)
                    this.ui.recognitionSelectContainer.hide();

                if (this.options.hide_noise)
                    this.ui.noiseContainer.hide();

                if (!this.options.hide_faces)
                    this.showFaces();

                var updateHeight = function () {
                    if (self.isDestroyed)
                        $(window).off('resize', updateHeight);
                    else
                        self.setHeight();
                };

                if ($.isNumeric(this.options.height))
                    this.setHeight(this.options.height);
                else
                    $(window).on('resize', updateHeight);

                // set current language
                api.getRobotLang(function (language) {
                    self.changeLanguage(language);
                });

                // set current speech recognition method
                api.getRosParam('/' + api.config.robot + '/webui/speech_recognition', function (method) {
                    self.setRecognitionMethod(method);
                });

                this.ui.noiseSlider.slider({
                    range: "min",
                    min: 0,
                    animate: true,
                    max: 5000,
                    value: 500,
                    change: function (e, ui) {
                        self.onNoiseEnergyChange(ui.value);
                    },
                    slide: function (e, ui) {
                        self.ui.noiseValue.html(ui.value);
                    }
                });
                api.getRosParam('/' + api.config.robot + '/recorder/energy_threshold', function (value) {
                    self.ui.noiseSlider.slider('value', value);
                });

                this.setUpKeyShortcuts();
            },
            /**
             * Accept or decline the last operator suggestion
             */
            setUpKeyShortcuts: function () {
                var self = this,
                    keyDown = function (e) {
                        if (self.isDestroyed) {
                            // remove event when view is destroyed
                            $(window).off('keydown', keyDown);
                            return;
                        }

                        var keyCode = e.keyCode;
                        // either DEL [ ] or F1-F12
                        if (self.operator_mode_enabled) {
                            if (_.contains([46, 219, 221], keyCode)) {
                                var last = self.collection.popLastSuggestion();
                                if (last) {
                                    e.preventDefault();
                                    switch (keyCode) {
                                        case 46:
                                            var data = last.toJSON();
                                            data['type'] = 'gibberish';
                                            api.logChatMessage(data);
                                            break;
                                        case 219:
                                            break;
                                        case 221:
                                            api.webSpeech(last.get('message'), app.language);
                                            break;
                                    }
                                }
                            } else if ((keyCode >= 112 && keyCode <= 123)) {
                                e.preventDefault();
                                var i = keyCode - 111,
                                    suggestions = self.collection.getSuggestions(),
                                    length = suggestions.length;

                                if (length >= i) {
                                    var model = suggestions[length - i];
                                    api.webSpeech(model.get('message'), app.language);
                                    self.collection.remove(model);
                                }
                            }
                        }
                    };

                $(window).keydown(keyDown);
            },
            showFaces: function () {
                var self = this;
                if (!this.facesView) {
                    this.facesView = new FacesView();
                    this.facesView.on('toggle', function () {
                        self.setHeight();
                    });
                    this.ui.facesContainer.html(this.facesView.render().$el);
                    this.on('destroy', function () {
                        self.facesView.destroy();
                    });
                }
            },
            onAttach: function () {
                this.ui.scrollbar.perfectScrollbar({wheelPropagation: true, swipePropagation: true});
                this.setHeight();
            },
            addListeners: function () {
                var self = this,
                    responseCallback = function (msg) {
                        if (self.isDestroyed) {
                            api.topics.tts['default'].unsubscribe(responseCallback);
                            api.topics.tts['en'].unsubscribe(responseCallback);
                            api.topics.tts['zh'].unsubscribe(responseCallback);
                        } else
                            self.responseCallback(msg);
                    },
                    speechActiveCallback = function (msg) {
                        if (self.isDestroyed)
                            api.topics.speech_active.unsubscribe(speechActiveCallback);
                        else
                            self.speechActiveCallback(msg);
                    },
                    voiceRecognised = function (msg) {
                        if (self.isDestroyed) {
                            api.topics.speech_topic.unsubscribe(voiceRecognised);
                            api.topics.speech_topic.removeAllListeners();
                        } else
                            self.voiceRecognised(msg);
                    },
                    suggestionCallback = function (msg) {
                        if (self.isDestroyed) {
                            api.topics.chatbot_responses['default'].unsubscribe(suggestionCallback)
                            api.topics.chatbot_responses['en'].unsubscribe(suggestionCallback)
                            api.topics.chatbot_responses['zh'].unsubscribe(suggestionCallback)
                        } else
                            self.suggestionCallback(msg);
                    },
                    operatorModeCallback = function (response) {
                        if (self.isDestroyed)
                            api.topics.selected_tts_mux.unsubscribe(operatorModeCallback);
                        else {
                            self.operator_mode_enabled = response.data == 'web_responses';
                            self.operatorModeSwitched();
                        }
                    };
                // tts callbacks
                api.topics.tts['default'].subscribe(responseCallback);
                api.topics.tts['en'].subscribe(responseCallback);
                api.topics.tts['zh'].subscribe(responseCallback);
                // speech events
                api.topics.speech_active.subscribe(speechActiveCallback);
                // user message callback
                api.topics.speech_topic.subscribe(voiceRecognised);

                // callbacks for response suggestions
                api.topics.chatbot_responses['default'].subscribe(suggestionCallback);
                api.topics.chatbot_responses['en'].subscribe(suggestionCallback);
                api.topics.chatbot_responses['zh'].subscribe(suggestionCallback);

                // fallow tts input topic to distinguish between operator and regular modes
                api.topics.selected_tts_mux.subscribe(operatorModeCallback);
            },
            setHeight: function (height) {
                if ($.isNumeric(height))
                    this.options.height = height;
                else {
                    if ($.isNumeric(this.options.height))
                        height = this.options.height;
                    else
                        height = app.LayoutInstance.getContentHeight();
                }

                // setting min height height
                height = Math.max(250, height - this.ui.footer.innerHeight());
                this.ui.scrollbar.css('height', height).perfectScrollbar('update');
            },
            operatorModeSwitched: function () {
                var self = this;
                (function (hide) {
                    _.each(self.collection.getSuggestions(), function (message) {
                        message.set('hidden', hide, {silent: false});
                    });
                })(!this.operator_mode_enabled);
            },
            suggestionCallback: function (msg) {
                if (this.operator_mode_enabled) {
                    var attrs = {author: 'Robot', message: msg.data, type: 'suggestion'};
                    this.collection.add(attrs);
                    api.logChatMessage(attrs);
                }
            },
            responseCallback: function (msg) {
                var attrs = {author: 'Robot', message: msg.data};
                this.collection.add(attrs);
                api.logChatMessage(attrs);
            },
            speechActiveCallback: function (msg) {
                if (this.speechEnabled) {
                    if (msg.data == 'start') {
                        this.speechPaused = true;
                        this.disableSpeech();
                    }
                } else if ((msg.data == 'stop') && this.speechPaused) {
                    this.enableSpeech();
                }
            },
            onSpeechEnabled: function () {
                this.speechEnabled = true;
                this.ui.recordButton.removeClass('btn-info').addClass('btn-danger');
            },
            onSpeechDisabled: function () {
                this.speechEnabled = false;
                if (typeof this.ui.recordButton.removeClass == 'function')
                    this.ui.recordButton.removeClass('btn-danger').addClass('btn-info').blur();
            },
            toggleSpeech: function (e) {
                var self = this;
                e.stopPropagation();
                e.preventDefault();
                var currentTime = new Date().getTime(),
                    maxClickTime = 500;

                if (e.type == 'touchstart') {
                    console.log('touch start');
                    self.touchstarted = currentTime;
                }
                if (e.type == 'touchend') {
                    console.log('touch end');
                    if (currentTime - maxClickTime < self.touchstarted) {
                        return;
                    }
                }
                if (this.speechEnabled) {
                    self.speechPaused = false;
                    this.disableSpeech(e);
                } else {
                    this.enableSpeech(e);
                }
            },
            messageKeyUp: function (e) {
                if (e.keyCode == 13) // submit message on enter
                    this.sendClicked();
            },
            sendClicked: function () {
                var message = this.ui.messageInput.val();
                if (message != '') {
                    api.sendChatMessage(message);
                    api.loginfo('[CLICK ACTION][CHAT] ' + message);
                }
                this.ui.messageInput.val('');
            },
            shutUpClicked: function () {
                api.shutUp();
            },
            rateGoodClicked: function () {
                api.sendChatMessage("gd");
            },
            rateBadClicked: function () {
                api.sendChatMessage("bd");
            },
            attachHtml: function (collectionView, childView) {
                var self = this;

                childView.$el.hide();
                collectionView._insertAfter(childView);

                $(childView.$el).fadeIn(400, function () {
                    self.scrollToChatBottom();
                });
            },
            scrollToChatBottom: function () {
                this.ui.scrollbar.stop().animate({scrollTop: this.ui.messages.height()}, 'slow', 'swing');
                this.ui.scrollbar.perfectScrollbar('update');
            },
            voiceRecognised: function (msg) {
                var attrs = {author: 'Me', message: msg.utterance};
                this.collection.add(attrs);
                api.logChatMessage(attrs);
            },
            enableSpeech: function () {
                var self = this;

                api.getRosParam('/' + api.config.robot + '/webui/speech_recognition', function (method) {
                    self.speech_recognition = method;
                    if (method == 'iflytek') self.enableIFlyTek();
                    else if (method == 'webspeech') self.enableWebspeech();
                });
            },
            disableSpeech: function () {
                if (this.speech_recognition == 'iflytek') {
                    this.disableIFlyTek();
                } else if (this.speech_recognition == 'webspeech') {
                    this.disableWebspeech()
                }

                this.speech_recognition = null;
            },
            enableIFlyTek: function () {
                var self = this;

                api.setDynParam('/' + api.config.robot + '/recorder', 'recording', true, {
                    success: function () {
                        self.onSpeechEnabled();
                    },
                    error: function () {
                        console.log('error enabling iflytek speech recognition')
                    }
                });
            },
            disableIFlyTek: function () {
                var self = this;

                api.setDynParam('/' + api.config.robot + '/recorder', 'recording', false, {
                    success: function () {
                        self.onSpeechDisabled();
                    },
                    error: function () {
                        console.log('error turning off iflytek speech recognition');
                    }
                });
            },
            languageButtonClick: function (e) {
                var language = $(e.target).data('lang');
                this.changeLanguage(language);
            },
            language: 'en',
            changeLanguage: function (language) {
                if (this.language == language) return;
                this.disableSpeech();

                this.changeMessageLanguage(language);
                this.language = language;
                app.language = language;

                this.ui.languageButton.removeClass('active');
                $('[data-lang="' + language + '"]', this.el).addClass('active');

                api.setRobotLang(this.language);
            },
            changeMessageLanguage: function (language) {
                if (!this.messages) this.messages = {};

                this.messages[this.language] = this.collection.clone();
                this.collection.reset();

                if (this.messages[language]) this.collection.add(this.messages[language].models);
            },
            enableWebspeech: function () {
                var self = this;

                if (annyang) {
                    annyang.abort();
                    annyang.removeCommands();
                    annyang.removeCallback();
                    annyang.setLanguage(this.language == 'zh' ? 'zh-CN' : 'en-US');
                    annyang.addCallback('start', function () {
                        console.log('starting speech recognition');
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'start'}));
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechstart'}));

                        self.onSpeechEnabled();
                    });
                    annyang.addCallback('end', function () {
                        console.log('end of speech');
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechend'}));
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'end'}));

                        self.onSpeechDisabled();
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
                }
            },
            disableWebspeech: function () {
                if (annyang) annyang.abort();
            },
            recognitionButtonClick: function (e) {
                this.setRecognitionMethod($(e.target).data('method'));
            },
            setRecognitionMethod: function (method) {
                // set default
                if (!method) method = 'webspeech';
                this.disableSpeech();

                this.ui.recognitionMethodButton.removeClass('active');
                $('[data-method="' + method + '"]', this.el).addClass('active');

                // update param
                api.setRosParam('/' + api.config.robot + '/webui/speech_recognition', method);

                if (method == 'dragon') this.setDragonInterval();
                else clearInterval(this.dragonInterval);
            },
            setDragonInterval: function () {
                var self = this,
                    value = null,
                    lastTime = null;

                this.dragonInterval = setInterval(function () {
                    if (self.isDestroyed) clearInterval(self.dragonInterval);
                    var current = self.ui.messageInput.val();

                    if (current) {
                        if (value == current) {
                            if (Date.now() - lastTime >= 1000) {
                                value = null;
                                self.sendClicked();
                            }
                        } else {
                            lastTime = Date.now();
                            value = current;
                        }
                    }
                }, 100);
            },
            adjustButtonClick: function (e) {
                var self = this;
                this.ui.adjustNoiseButton.blur();

                api.getRosParam('/' + api.config.robot + '/webui/speech_recognition', function (method) {
                    if (method == 'iflytek') {
                        api.setDynParam('/' + api.config.robot + '/recorder', 'adjust_noise', true, {
                            success: function () {
                                self.ui.adjustNoiseButton.removeClass('active');
                                api.getRosParam('/' + api.config.robot + '/recorder/energy_threshold', function (value) {
                                    console.log('Ambient noise threshold ' + value);
                                    self.ui.noiseSlider.slider('value', value);
                                });
                            },
                            error: function () {
                                console.log('error adjusting ambient noise')
                            }
                        });
                    }
                });
            },
            onNoiseEnergyChange: function (value) {
                var self = this;
                api.getRosParam('/' + api.config.robot + '/webui/speech_recognition', function (method) {
                    if (method == 'iflytek') {
                        api.setDynParam('/' + api.config.robot + '/recorder', 'energy_threshold', value, {
                            success: function () {
                                self.ui.noiseValue.html(value);
                                console.log('Change noise energy');
                            },
                            error: function () {
                                console.log('error changing noise energy threshold')
                            }
                        });
                    }
                });
            }
        });
    });

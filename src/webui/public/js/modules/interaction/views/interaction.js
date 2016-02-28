define(['application', "marionette", './message', "tpl!./templates/interaction.tpl", 'lib/api', '../entities/message_collection',
        'jquery', './faces', 'underscore', 'scrollbar'],
    function (app, Marionette, MessageView, template, api, MessageCollection, $, FacesView, _) {
        return Marionette.CompositeView.extend({
            template: template,
            childView: MessageView,
            childViewContainer: '.app-messages',
            ui: {
                messages: '.app-messages',
                recordButton: '.app-record-button',
                messageInput: '.app-message-input',
                sendButton: '.app-send-button',
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
                facesContainer: '.app-faces-container'
            },
            events: {
                'touchstart @ui.recordButton': 'toggleSpeech',
                'touchend @ui.recordButton': 'toggleSpeech',
                'click @ui.recordButton': 'toggleSpeech',
                'keyup @ui.messageInput': 'messageKeyUp',
                'click @ui.sendButton': 'sendClicked',
                'click @ui.languageButton': 'languageButtonClick',
                'click @ui.recognitionMethodButton': 'recognitionButtonClick',
                'click @ui.adjustNoiseButton': 'adjustButtonClick'
            },
            childViewOptions: function () {
                return {
                    collection: this.collection
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
                this.speechStarted = 0;

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
                    keyPress = function (e) {
                        var suggestions = self.collection.getSuggestions(),
                            length = suggestions.length;

                        if (self.isDestroyed) // remove event when view is destroyed
                            $(window).off('keypress', keyPress);

                        if (length > 0 && _.contains([91, 93], e.keyCode)) {
                            e.preventDefault();

                            if (e.keyCode == 91) {
                                self.collection.clearSuggestions();
                            } else if (e.keyCode == 93) {
                                api.webSpeech(suggestions[length - 1].get('message'), app.language);
                                self.collection.clearSuggestions();
                            }
                        }
                    };

                $(window).keypress(keyPress);
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
                this.ui.scrollbar.perfectScrollbar();
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
                height = Math.max(250, height - this.ui.footer.outerHeight())

                this.ui.scrollbar.css('height', height).perfectScrollbar('update');
            },
            operatorModeSwitched: function () {
                var self = this;
                if (!this.operator_mode_enabled)
                    _.each(this.collection.where({type: 'suggestion'}), function (msg) {
                        self.collection.remove(msg);
                    });
            },
            suggestionCallback: function (msg) {
                if (this.operator_mode_enabled)
                    this.collection.add({author: 'Robot', message: msg.data, type: 'suggestion'});
            },
            responseCallback: function (msg) {
                this.collection.clearSuggestions();
                this.collection.add({author: 'Robot', message: msg.data});
            },
            speechActiveCallback: function (msg) {
                if (this.speechEnabled) {
                    if (msg.data == 'start') {
                        this.speechPaused = true;
                        this.disableSpeech();
                    }
                } else if ((msg.data != 'start') && this.speechPaused) {
                    this.enableSpeech()
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
                // submit on enter press
                if (e.keyCode == 13) this.ui.sendButton.click();
            },
            sendClicked: function () {
                var message = this.ui.messageInput.val();
                if (message != '') api.sendChatMessage(message);
                this.ui.messageInput.val('');
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
                var self = this;

                if (!this.scrolling)
                    this.ui.scrollbar.animate({scrollTop: this.ui.messages.height()}, 'slow', 'swing', function () {
                        self.scrolling = false;
                    });
                this.scrolling = true;
            },
            voiceRecognised: function (message) {
                this.collection.add({author: 'Me', message: message.utterance});
            },
            enableSpeech: function () {
                var self = this;

                api.getRosParam('/' + api.config.robot + '/webui/speech_recognition', function (method) {
                    if (method == 'iflytek') {
                        self.speech_recognition = method;
                        self.enableIFlyTek();
                    } else {
                        self.speech_recognition = 'webspeech';
                        self.enableWebspeech();
                    }
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

                if (!this.speechRecognition || !this.speechEnabled) {
                    if ('webkitSpeechRecognition' in window) {
                        this.speechRecognition = new webkitSpeechRecognition();
                    } else if ('SpeechRecognition' in window) {
                        this.speechRecognition = new SpeechRecognition();
                    } else {
                        console.log('webspeech api not supported');
                        this.speechRecognition = null;
                    }

                    this.speechRecognition.lang = this.language == 'zh' ? 'cmn-Hans-CN' : 'en-US';
                    this.speechRecognition.interimResults = false;
                    this.speechRecognition.continuous = false;

                    this.speechRecognition.onstart = function () {
                        console.log('starting webspeech');
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'start'}));
                        self.onSpeechEnabled();
                    };
                    this.speechRecognition.onspeechstart = function () {
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechstart'}));
                    };
                    this.speechRecognition.onspeechend = function () {
                        api.topics.chat_events.publish(new ROSLIB.Message({data: 'speechend'}));
                    };
                    this.speechRecognition.onresult = function (event) {
                        var mostConfidentResult = null;

                        _.each(event.results[event.results.length - 1], function (result) {
                            if ((!mostConfidentResult || mostConfidentResult.confidence <= result.confidence))
                                mostConfidentResult = result;
                        });

                        if (mostConfidentResult)
                            api.sendChatMessage(mostConfidentResult.transcript);
                    };

                    this.speechRecognition.onerror = function (event) {
                        switch (event.error) {
                            case 'not-allowed':
                            case 'service-not-allowed':
                                self.onSpeechDisabled();
                                break;
                        }
                        console.log('error recognising speech');
                        console.log(event);

                    };
                    this.speechRecognition.onend = function () {
                        if (self.speechEnabled) {
                            var timeSinceLastStart = new Date().getTime() - self.speechStarted;
                            if (timeSinceLastStart < 1000) {
                                setTimeout(function () {
                                    self.speechRecognition.start();
                                }, 1000);
                            } else {
                                self.speechRecognition.start();
                            }
                        } else {
                            console.log('end of speech');
                            api.topics.chat_events.publish(new ROSLIB.Message({data: 'end'}));
                        }
                    };
                    this.speechStarted = new Date().getTime();
                    this.speechRecognition.start();
                }
            },
            disableWebspeech: function () {
                if (this.speechRecognition) {
                    this.onSpeechDisabled();
                    this.speechRecognition.stop();
                    this.speechRecognition = null;
                }
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

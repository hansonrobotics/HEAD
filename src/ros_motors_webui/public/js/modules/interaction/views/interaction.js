define(["application", './message', "tpl!./templates/interaction.tpl", 'lib/api', 'RecordRTC', 'underscore'],
    function (App, MessageView, template, api, RecordRTC, _) {
        var self;
        App.module("Interaction.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Interaction = Marionette.CompositeView.extend({
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
                    faceThumbnails: '.app-face-thumbnails',
                    faceContainer: '.app-select-person-container',
                    faceCollapse: '.app-face-container',
                    footer: 'footer',
                    languageButton: '.app-language-select button'
                },
                events: {
                    'touchstart @ui.recordButton': 'toggleSpeech',
                    'touchend @ui.recordButton': 'toggleSpeech',
                    'click @ui.recordButton': 'toggleSpeech',
                    'keyup @ui.messageInput': 'messageKeyUp',
                    'click @ui.sendButton': 'sendClicked',
                    'click @ui.languageButton': 'changeLanguage'
                },
                initialize: function () {
                    self = this;
                    api.enableInteractionMode();
                    api.topics.chat_responses.subscribe(this.responseCallback);
                    api.topics.speech_active.subscribe(this.speechActiveCallback);
                    this.speechPaused = false
                },
                onDestroy: function () {
                    this.options.faceCollection.unsubscribe();
                    api.topics.chat_responses.unsubscribe(this.responseCallback);
                    api.topics.speech_active.unsubscribe(this.speechActiveCallback);
                    this.disableWebspeech();
                    this.disableAudioRecording();
                },
                updateFaces: function () {
                    var currentTime = new Date().getTime();

                    // remove lost faces older than 3 seconds
                    $('img', this.ui.faceThumbnails).each(function (i, img) {
                        var id = parseInt($(img).attr('title'));

                        if (!self.options.faceCollection.findWhere({id: id}) && (currentTime - $(img).data('time-added')) > 3000) {
                            $(img).remove();

                            if (self.options.faceCollection.getLookAtFaceId() == id)
                                self.ui.faceCollapse.collapse('show');
                        }
                    });

                    this.options.faceCollection.each(function (face) {
                        var img = $('img[title="' + face.get('id') + '"]', self.ui.faceThumbnails),
                        // update thumbnail every 3 seconds, update time added
                            thumbnailUrl = face.getThumbnailUrl() + '?' + parseInt(currentTime / 3000);

                        // if image already shown
                        if (img.length > 0) {
                            $(img).prop({
                                src: thumbnailUrl
                            }).data('time-added', currentTime);
                        } else {
                            // create new thumbnail
                            var setActiveThumbnail = function (el) {
                                    $('img', self.ui.faceThumbnails).removeClass('active');
                                    $(el).addClass('active');
                                },
                                el = $('<img>').prop({
                                    src: thumbnailUrl,
                                    title: face.get('id'),
                                    'class': 'face-thumbnail thumbnail',
                                    width: 100,
                                    height: 100
                                }).data('time-added', currentTime).click(function () {
                                    self.options.faceCollection.setLookAtFaceId(face.get('id'));
                                    setActiveThumbnail(this);
                                });

                            if (self.options.faceCollection.getLookAtFaceId() == face.get('id'))
                                setActiveThumbnail(el);

                            self.ui.faceThumbnails.append(el);
                        }
                    });

                    if ($('img', this.ui.faceThumbnails).length == 0 && this.options.faceCollection.isEmpty()) {
                        if (!this.facesEmpty) {
                            this.facesEmpty = true;
                            this.ui.faceContainer.slideUp();
                        }
                    } else if (typeof this.facesEmpty == 'undefined' || this.facesEmpty) {
                        this.facesEmpty = false;

                        this.ui.faceCollapse.removeClass('in');
                        this.ui.faceContainer.slideDown();
                    }
                },
                serializeData: function () {
                    return {
                        faces: this.options.faceCollection
                    };
                },
                onRender: function () {
                    this.options.faceCollection.on('change', this.updateFaces, this);
                    this.options.faceCollection.subscribe();
                    this.updateFaces();

                    // update chat margins on face collapse show/hide
                    this.ui.faceCollapse.on('shown.bs.collapse hidden.bs.collapse', function () {
                        self.ui.messages.css('margin-bottom', self.ui.footer.height());
                        self.scrollToChatBottom();
                    });
                },
                responseCallback: function (msg) {
                    self.collection.add({author: 'Robot', message: msg.data});
                },
                speechActiveCallback: function (msg) {
                    if (self.speechEnabled) {
                        if (msg.data == 'start') {
                            self.speechPaused = true;
                            self.disableSpeech();
                        }
                    } else if ((msg.data != 'start') && self.speechPaused) {
                        self.enableSpeech()
                    }
                },
                enableSpeech: function () {
                    if (!this.speechEnabled) {
                        switch (this.language) {
                            case 'en':
                                this.enableWebspeech();
                                break;
                            case 'zh':
                                this.enableAudioRecording();
                        }
                    }
                },
                disableSpeech: function () {
                    if (this.speechEnabled) {
                        switch (this.language) {
                            case 'en':
                                this.pauseWebspeech();
                                break;
                            case 'zh':
                                this.disableAudioRecording();
                        }
                    }
                },
                onSpeechEnabled: function () {
                    self.speechEnabled = true;
                    self.ui.recordButton.removeClass('btn-info').addClass('btn-danger');
                },
                onSpeechDisabled: function () {
                    self.speechEnabled = false;
                    if (typeof self.ui.recordButton.removeClass == 'function')
                        self.ui.recordButton.removeClass('btn-danger').addClass('btn-info').blur();
                },
                toggleSpeech: function (e) {
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
                        this.disableSpeech(e);
                    } else {
                        this.enableSpeech(e);
                    }
                },
                messageKeyUp: function (e) {
                    if (e.keyCode == 13)
                        this.ui.sendButton.click();
                },
                sendClicked: function () {
                    var message = this.ui.messageInput.val();

                    if (message != '') {
                        this.collection.add({author: 'Me', message: message});
                        api.sendChatMessage(message);
                    }

                    this.ui.messageInput.val('');
                },
                attachHtml: function (collectionView, childView) {
                    childView.$el.hide();
                    collectionView._insertAfter(childView);

                    $(childView.$el).fadeIn(400, function () {
                        self.scrollToChatBottom();
                    });
                },
                scrollToChatBottom: function () {
                    if (!self.scrolling)
                        $('html, body').animate({scrollTop: $(document).height()}, 'slow', 'swing', function () {
                            self.scrolling = false;
                        });
                    self.scrolling = true;
                },
                sendMessage: function (message) {
                    self.collection.add({author: 'Me', message: message});

                    var chat_message = new ROSLIB.Message({
                        utterance: message,
                        confidence: 99
                    });

                    api.topics.speech_topic.publish(chat_message);
                },
                hello: function () {
                    self.sendMessage('hello');
                },
                bye: function () {
                    self.sendMessage('bye');
                },
                enableWebspeech: function () {
                    if (!this.speechRecognition) {
                        var mostConfidentResult = null;
                        if ('webkitSpeechRecognition' in window) {
                            this.speechRecognition = new webkitSpeechRecognition();
                        } else if ('SpeechRecognition' in window) {
                            this.speechRecognition = new SpeechRecognition();
                        } else {
                            console.log('webspeech api not supported');
                            this.speechRecognition = null;
                        }

                        this.speechRecognition.interimResults = true;
                        this.speechRecognition.continuous = true;

                        this.speechRecognition.onstart = function () {
                            console.log('starting webspeech');
                            self.onSpeechEnabled();
                        };

                        this.speechRecognition.onresult = function (event) {
                            _.each(event.results, function (results) {
                                _.each(results, function (result) {
                                    if (!mostConfidentResult || mostConfidentResult.confidence <= result.confidence)
                                        mostConfidentResult = result;
                                });
                            });
                        };

                        this.speechRecognition.onerror = function (event) {
                            console.log('error recognising speech');
                            console.log(event);
                            self.onSpeechDisabled();
                        };
                        this.speechRecognition.onend = function () {
                            console.log('end of speech');
                            self.onSpeechDisabled();
                            if (mostConfidentResult && mostConfidentResult.confidence > 0.5)
                                self.sendMessage(mostConfidentResult.transcript);

                            mostConfidentResult = null;
                        };
                    }

                    if (this.speechRecognition && !this.speechEnabled)
                        this.speechRecognition.start();

                    return this.speechRecognition;
                },
                pauseWebspeech: function () {
                    if (this.speechRecognition)
                        this.speechRecognition.stop();
                },
                disableWebspeech: function () {
                    if (this.speechRecognition) {
                        this.speechRecognition.abort();
                        this.speechRecognition = null;
                    }
                },
                enableAudioRecording: function () {
                    var session = {
                        audio: true
                    }, onError = function () {
                        console.log('error recording');
                        self.onSpeechDisabled();
                    };

                    this.recordRTC = null;
                    navigator.getUserMedia(session, function (mediaStream) {
                        self.onSpeechEnabled();

                        self.recordRTC = RecordRTC(mediaStream, {
                            type: 'audio',
                            sampleRate: '44100',
                            numberOfAudioChannels: 1
                        });
                        self.recordRTC.startRecording();
                    }, onError);
                },
                disableAudioRecording: function () {
                    if (this.recordRTC)
                        this.recordRTC.stopRecording(function () {
                            self.onSpeechDisabled();

                            var formData = new FormData();
                            formData.append('audio', self.recordRTC.getBlob());

                            $.ajax({
                                type: 'POST',
                                url: '/chat_audio',
                                data: formData,
                                contentType: false,
                                cache: false,
                                processData: false
                            }).done(function (res) {
                                if (res.success == true) {
                                    api.topics.voice.publish(new ROSLIB.Message({data: res.filename}))
                                } else {
                                    console.log("Error while saving file")
                                }
                            });
                        });
                },
                language: 'en',
                changeLanguage: function (e) {
                    var language = $(e.target).data('lang');

                    this.changeMessageLanguage(language);
                    this.disableSpeech();

                    this.language = language;
                    this.ui.languageButton.removeClass('active');
                    $(e.target).addClass('active');

                    api.setRobotLang(this.language);
                },
                changeMessageLanguage: function (language) {
                    if (this.language == language) return;

                    if (!this.messages)
                        this.messages = {};

                    this.messages[this.language] = this.collection.clone();

                    this.collection.reset();
                    if (this.messages[language])
                        this.collection.add(this.messages[language].models);
                }
            });
        });

        return App.module('Interaction.Views').Interaction;
    });

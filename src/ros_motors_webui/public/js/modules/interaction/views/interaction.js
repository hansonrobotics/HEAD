define(["application", './message', "tpl!./templates/interaction.tpl", 'lib/api', 'annyang', 'jquery'],
    function (App, MessageView, template, api, annyang, $) {
        var self;
        App.module("Interaction.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Interaction = Marionette.CompositeView.extend({
                template: template,
                childView: MessageView,
                childViewContainer: '.app-messages',
                ui: {
                    recordButton: '.app-record-button',
                    messageInput: '.app-message-input',
                    sendButton: '.app-send-button',
                    unsupported: '.app-unsupported',
                    supported: '.app-supported',
                    recordContainer: '.record-container',
                    faceThumbnails: '.app-face-thumbnails',
                    faceContainer: '.app-select-person-container'
                },
                events: {
                    'click @ui.recordButton': 'recognizeSpeech',
                    'keyup @ui.messageInput': 'messageKeyUp',
                    'click @ui.sendButton': 'sendClicked'
                },
                onDestroy: function () {
                    options.faceCollection.unsubscribe();

                    if (annyang)
                        annyang.abort();

                    clearTimeout(self.keepAliveInterval);
                },
                updateFaces: function () {
                    var self = this;

                    if (this.options.faceCollection.isEmpty()) {
                        this.ui.faceContainer.hide();
                        this.ui.recordButton.fadeIn();
                    } else {
                        this.ui.faceContainer.fadeIn();
                        this.ui.recordButton.hide();

                        this.ui.faceThumbnails.html('');
                        this.options.faceCollection.each(function (face) {
                            self.ui.faceThumbnails.append($('<img>').prop({
                                src: face.getThumbnailUrl() + '?' + parseInt(new Date().getTime() / 5000),
                                title: face.get('id'),
                                'class': 'face-thumbnail thumbnail',
                                width: 100,
                                height: 100
                            }));
                        });
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

                    var self = this;
                    var responseCallback = function (msg) {
                            self.collection.add({author: 'Robot', message: msg.data});
                        },
                        speechActiveCallback = function (msg) {
                            if (msg.data == 'start') {
                                if (window.location.protocol != "https:") {
                                    if (annyang) annyang.pause();
                                } else {
                                    if (annyang) annyang.abort();
                                }
                            } else {
                                if (annyang) annyang.resume();
                            }
                        };

                    api.topics.chat_responses.subscribe(responseCallback);
                    api.topics.speech_active.subscribe(speechActiveCallback);

                    if (annyang) {
                        annyang.start();

                        var commands = {
                            'hi *': this.hello,
                            'hello *': this.hello,
                            'bye *': this.bye,
                            '*text': this.sendMessage
                        };
                        annyang.addCommands(commands);
                        // keeps speech alive for mobile devices if they went sleep or switched app.
                        this.keepAlive();
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
                keepAlive: function () {
                    this.keepAliveInterval = setInterval(function () {
                        if (annyang)
                            annyang.start();
                    }, 10000);
                },
                attachHtml: function (collectionView, childView, index) {
                    var self = this;

                    childView.$el.hide();
                    collectionView._insertAfter(childView);

                    $(childView.$el).fadeIn(400, function () {
                        if (!self.scrolling)
                            $('html, body').animate({scrollTop: $(document).height()}, 'slow', 'swing', function () {
                                self.scrolling = false;
                            });

                        self.scrolling = true;
                    });
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
                recognizeSpeech: function () {
                    alert('Say Hi to start');
                }
            });
        });

        return App.module('Interaction.Views').Interaction;
    })
;

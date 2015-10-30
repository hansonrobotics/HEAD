define(["application", './message', "tpl!./templates/interaction.tpl", 'lib/api', 'annyang'],
    function (App, MessageView, template, api, annyang) {
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
                    footer: 'footer'
                },
                events: {
                    'mousedown @ui.recordButton': 'enableSpeech',
                    'mouseup @ui.recordButton': 'disableSpeech',
                    'mouseleave @ui.recordButton': 'disableSpeech',
                    'keyup @ui.messageInput': 'messageKeyUp',
                    'click @ui.sendButton': 'sendClicked'
                },
                initialize: function () {
                    self = this;
                },
                onDestroy: function () {
                    this.options.faceCollection.unsubscribe();
                    this.disableSpeech();
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
                },
                enableSpeech: function () {
                    if (annyang && !this.speechEnabled) {
                        this.ui.recordButton.tooltip({
                            placement: 'left',
                            title: 'Release to stop'
                        }).removeClass('btn-info').addClass('btn-danger');

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
                        this.speechEnabled = true;
                    }
                },
                disableSpeech: function () {
                    if (this.speechEnabled) {
                        this.ui.recordButton.tooltip('destroy').removeClass('btn-danger').addClass('btn-info').blur();

                        if (annyang)
                            annyang.abort();

                        clearTimeout(this.keepAliveInterval);
                        this.speechEnabled = false;
                    }
                },
                toggleSpeech: function () {
                    this.ui.recordButton.blur();

                    if (this.speechEnabled) {
                        this.disableSpeech();
                    } else {
                        this.enableSpeech();
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
                }
            });
        });

        return App.module('Interaction.Views').Interaction;
    })
;

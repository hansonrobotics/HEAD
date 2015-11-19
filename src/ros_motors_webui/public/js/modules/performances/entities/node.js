define(['application', 'lib/api', 'lib/web_speech_api'], function (App, api, WebSpeechApi) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Node = Backbone.Model.extend({
            call: function () {
                var self = this;
                switch (this.get('name')) {
                    case 'gesture':
                        api.setGesture(this.get('gesture'), 1, this.get('speed'), this.get('magnitude'));
                        break;
                    case 'emotion':
                        api.setEmotion(this.get('emotion'), this.get('magnitude'), parseFloat(this.get('duration')));
                        break;
                    case 'look_at':
                        api.setFaceTarget(3, this.get('x'), -this.get('y'));
                        break;
                    case 'gaze_at':
                        api.setGazeTarget(3, this.get('x'), -this.get('y'));
                        break;
                    case 'speech':
                        api.robotSpeech(this.get('text'));
                        break;
                    case 'interaction':
                        api.enableInteractionMode();
                        break;
                    case 'pause':
                        this.trigger('pause');
                        break;
                    case 'speech_input':
                        this.trigger('pause');

                        api.enableRecording(function () {
                            var speechEvent = function (event) {
                                console.log(event);
                                
                                // resume on any event
                                self.trigger('resume');
                                api.topics.speech_active.unsubscribe(speechEvent);
                                api.disableRecording();
                            };
                            api.topics.speech_active.subscribe(speechEvent);
                        }, function () {
                            // resume on error
                            self.trigger('resume');
                        });
                        break;
                }
            },
            finish: function () {
                switch (this.get('name')) {
                    case 'interaction':
                        api.disableInteractionMode();
                        break;
                }
            },
            toJSON: function () {
                var json = Backbone.Model.prototype.toJSON.call(this);
                if (this.get('el')) delete json['el'];

                return json;
            },
            destroy: function () {
                // remove an associated element
                if (this.get('el'))
                    $(this.get('el')).remove();

                Backbone.Model.prototype.destroy.call(this);
            },
            /**
             * Trigger pause event -> listen for speech -> resume when recognised
             */
            speechInput: function () {
                this.trigger('pause');

                var self = this,
                    speechRecognition = WebSpeechApi.getSpeechRecognition(),
                    lastResultTime = new Date().getTime(),
                    recognised,
                    checkForSpeech = setInterval(function () {
                        if (recognised && lastResultTime + 300 < new Date().getTime()) {
                            speechRecognition.stop();
                            clearInterval(checkForSpeech);
                        }
                    }, 500);

                speechRecognition.continuous = true;
                speechRecognition.interimResults = true;

                speechRecognition.onresult = function (event) {
                    var mostConfidentResult;

                    _.each(event.results, function (results) {
                        _.each(results, function (result) {
                            if (!mostConfidentResult || mostConfidentResult.confidence <= result.confidence)
                                mostConfidentResult = result;
                        });
                    });

                    var currentTime = new Date().getTime();

                    if (mostConfidentResult && (!recognised || mostConfidentResult != recognised)) {
                        recognised = mostConfidentResult;
                        lastResultTime = currentTime;
                    }
                };

                speechRecognition.onend = function () {
                    self.trigger('resume');
                    if (recognised) api.sendChatMessage(recognised.transcript);
                };

                speechRecognition.start();
            }
        });
        Entities.NodeCollection = Backbone.Collection.extend({
            model: Entities.Node,
            comparator: 'start_time'
        });
    });
});

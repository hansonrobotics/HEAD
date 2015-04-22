define(['application', './views/interaction', 'lib/api', 'backbone', 'annyang'],
    function (App, InteractionView, api, Backbone, annyang) {
        var interaction = {
            index: function () {
                api.blenderMode.enable();
                api.topics.cmdTree.publish(new ROSLIB.Message({data: 'btree_on'}));
                api.setExpression("Neutral", 0);

                this.interactionView = new InteractionView();

                App.LayoutInstance.setTitle('Interaction');
                App.LayoutInstance.getRegion('content').show(this.interactionView);
                App.LayoutInstance.showNav();

                api.topics.chat_responses.subscribe(function (msg) {
                    interaction.addMessage('Robot', msg.data);
                });

                interaction.speech_active = true;
                api.topics.speech_active.subscribe(function (msg) {
                    if (msg.data == 'start') {
                        console.log('paused');
                        annyang.pause();
                    } else {
                        annyang.resume();
                    }
                    console.log('speech active');
                });

                $('#app-record-button').click(function () {
                    interaction.recognizeSpeech();
                });

                $('#app-message-input').keyup(function (e) {
                    if (e.keyCode == 13)
                        $('#app-send-button').click();
                });

                $('#app-send-button').click(function () {
                    if ($('#app-message-input').val() != "") {
                        interaction.addMessage('Me', $('#app-message-input').val());
                        api.sendChatMessage($('#app-message-input').val());
                        $('#app-message-input').val('');
                    }
                });

                annyang.start();
                annyang.debug();
                var commands = {
                    'hi (han)': interaction.hello,
                    'hello (han)': interaction.hello,
                    'hello (hun)': interaction.hello,
                    'hello (hon)': interaction.hello,
                    'hi (hun)': interaction.hello,
                    'hi (hon)': interaction.hello,
                    'bye *bye': interaction.bye,
                    '*text': interaction.voiceRecognized
                };
                annyang.addCommands(commands);
                interaction.started = false;
                interaction.checkSleep();
            },
            wakeUp: function () {
                annyang.start();
            },
            checkSleep: function () {
                var lastSync = new Date().getTime();
                setInterval(function () {
                    var now = new Date().getTime();
                    if ((now - lastSync) > 2000) {
                        interaction.wakeUp();
                    }
                }, 1000);
            },
            addMessage: function (name, message) {
                var element;

                if (name == 'Robot') {
                    element = $('#leftMsg').clone().removeAttr('id');
                } else {
                    element = $('#rightMsg').clone().removeAttr('id');
                }

                if (typeof this.scrolling == 'undefined')
                    this.scrolling = false;

                $(element).find('.msg').text(message);
                $(element).find('.name').text(name);
                $(element).find('.time').text(interaction.currentTime());
                $(element).hide();
                $('#app-chat').append(element);
                $(element).fadeIn(400, function () {
                    if (!interaction.scrolling)
                        $('html, body').animate({scrollTop: $(document).height()}, 'slow', 'swing', function () {
                            interaction.scrolling = false;
                        });

                    interaction.scrolling = true;
                });
            },
            currentTime: function () {
                var date = new Date();
                var hour = date.getHours();
                var min = date.getMinutes();

                if (min < 10)
                    min = "0" + min;

                var amPm = hour < 12 ? "am" : "pm";

                if (hour > 12)
                    hour = hour - 12;

                return hour + ":" + min + " " + amPm;
            },
            hello: function () {
                console.log('Conversation started');
                interaction.started = true;
                var chat_message = new ROSLIB.Message({
                    utterance: 'hello',
                    confidence: 99
                });
                api.topics.speech_topic.publish(chat_message);
            },
            bye: function () {
                if (interaction.started) {
                    console.log('Conversation Finished');
                    var chat_message = new ROSLIB.Message({
                        utterance: 'bye',
                        confidence: 99
                    });
                    api.topics.speech_topic.publish(chat_message);
                }
                interaction.started = false;
            },
            voiceRecognized: function (text) {
                console.log(text);
                if (interaction.started) {
                    var chat_message = new ROSLIB.Message({
                        utterance: text,
                        confidence: 99
                    });
                    api.topics.speech_topic.publish(chat_message);
                    interaction.addMessage('Me', text);
                }
            },
            recognizeSpeech: function () {
                alert('Say Hi to start');
            }
        };

        return interaction;
    });

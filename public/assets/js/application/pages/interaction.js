define(['jquery', './../lib/api', './gestures'], function ($, api, gestures) {
    var interaction = {
        init: function () {
            api.topics.chat_responses.subscribe(function (msg) {
                interaction.addMessage('Robot', msg.data);
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
        },
        loadPage: function () {
            var blenderMessage, blinkMessage, treeMessage;

            //blenderMessage = new ROSLIB.Message({data: 'TrackDev'});
            //api.topics.cmdBlender.publish(blenderMessage);

            api.blenderMode.enable();

            blinkMessage = new ROSLIB.Message({data: 'arthur:start'});
            api.topics.cmdBllink.publish(blinkMessage);

            treeMessage = new ROSLIB.Message({data: 'btree_on'});
            api.topics.cmdTree.publish(treeMessage);

            api.setExpression("happy", 0);
            api.pointHead({yaw: 0, pitch: 0, roll: 0});

            gestures.demo.enable()
        },
        recognizeSpeech: function () {
            if (!('webkitSpeechRecognition' in window)) {
                alert('Browser not supported. Use the newest Chrome browser');
            } else {
                var recognition = new webkitSpeechRecognition();

                recognition.onstart = function () {
                    document.querySelector('#app-record-button').textContent =
                        "Cancel recording";
                };

                recognition.onerror = function (event) {
                    alert('Recognition error');
                };

                recognition.onend = function () {
                    document.querySelector('#app-record-button').textContent =
                        "Start recording";
                };

                recognition.onresult = function (event) {
                    var utterance = event.results[0][0].transcript;
                    var chat_message = new ROSLIB.Message({
                        utterance: utterance,
                        confidence: Math.round(event.results[0][0].confidence * 100)
                    });
                    api.topics.speech_topic.publish(chat_message);
                    interaction.addMessage('Me', utterance);
                };
                recognition.start();
            }
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
        }
    };

    return interaction;
});

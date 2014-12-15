$(function () {
    // ROS object to connect to ROS
    var responses;

    function startTree() {
//        var cmdBlender = new ROSLIB.Topic({
//            ros: ros,
//            name: '/cmd_blendermode',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'Dummy'
//        });
//        cmdBlender.publish(msg);
//        var cmdBllink = new ROSLIB.Topic({
//            ros: ros,
//            name: '/arthur/cmd_blink',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'arthur:start'
//        });
//        cmdBllink.publish(msg);
//        var cmdTree = new ROSLIB.Topic({
//            ros: ros,
//            name: '/arthur/behavior_switch',
//            messageType: 'std_msgs/String'
//        });
//        var msg = new ROSLIB.Message({
//            data: 'btree_on'
//        });
//        cmdTree.publish(msg);
    }

    function recognizeSpeech() {
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
                RosUI.topics.speech_topic.publish(chat_message);
                addMessage('Me', utterance);
            };
            recognition.start();
        }
    }

    function ready() {
        responses = new ROSLIB.Topic({
            ros: ros,
            name: '/arthur/chatbot_responses',
            messageType: 'std_msgs/String'
        });
        // Publish the response
        responses.subscribe(function (msg) {
            addMessage('Robot', msg.data);
        });
    }

    // Find out exactly when we made a connection.
    RosUI.ros.$.on('connection', function () {
        ready();

        $('#app-record-button').click(function () {
            recognizeSpeech();
        });
        startTree();
    });
    // Create a connection to the rosbridge WebSocket server.
//    ros.connect(websocketAddress());

    function currentTime() {
        var d = new Date();
        var hr = d.getHours();
        var min = d.getMinutes();
        if (min < 10) {
            min = "0" + min;
        }
        var ampm = hr < 12 ? "am" : "pm";
        if (hr > 12) {
            hr = hr - 12;
        }
        return hr + ":" + min + " " + ampm;
    }
    var scrolling = false;
    function addMessage(name, message) {
        var el;
        if (name == 'Robot') {
            el = $('#leftMsg').clone().removeAttr('id');
        } else {
            el = $('#rightMsg').clone().removeAttr('id');
        }
        $(el).find('.msg').text(message);
        $(el).find('.name').text(name);
        $(el).find('.time').text(currentTime());
        $(el).hide();
        $('#app-chat').append(el);
        $(el).fadeIn(400, function() {
            if (! scrolling)
                $('html, body').animate({scrollTop: $(document).height()}, 'slow', 'swing', function() {
                    scrolling = false;
                });

            scrolling = true;
        });
    }

    $('#app-message-input').keyup(function(e){
        if(e.keyCode == 13)
            $('#app-send-button').click();
    });

    $('#app-send-button').click(function () {
        if ($('#app-message-input').val() != "") {
            addMessage('Me', $('#app-message-input').val());
            var chat_message = new ROSLIB.Message({
                utterance: $('#app-message-input').val(),
                confidence: Math.round(0.9 * 100)
            });
            $('#app-message-input').val('');
            RosUI.topics.speech_topic.publish(chat_message);
        }
    });
});


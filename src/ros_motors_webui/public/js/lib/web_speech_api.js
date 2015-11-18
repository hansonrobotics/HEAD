define([], function () {
    return {
        getSpeechRecognition: function () {
            var speechRecognition;
            if ('webkitSpeechRecognition' in window) {
                speechRecognition = new webkitSpeechRecognition();
            } else if ('SpeechRecognition' in window) {
                speechRecognition = new SpeechRecognition();
            } else {
                return null;
            }

            return speechRecognition;
        }
    };
});

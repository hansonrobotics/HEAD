define(['underscore'], function (_) {
    return {
        getInstance: function (lang) {
            if (!lang) lang = 'en-US';
            var speechRecognition = null;
            if ('webkitSpeechRecognition' in window)
                speechRecognition = new webkitSpeechRecognition();
            else if ('SpeechRecognition' in window)
                speechRecognition = new SpeechRecognition();

            speechRecognition.lang = lang;
            speechRecognition.interimResults = false;

            return speechRecognition;
        },
        getMostConfidentResult: function (results) {
            var mostConfidentResult = null;
            _.each(results[results.length - 1], function (result) {
                if ((!mostConfidentResult || mostConfidentResult.confidence <= result.confidence))
                    mostConfidentResult = result;
            });
            return mostConfidentResult;
        }
    };
});

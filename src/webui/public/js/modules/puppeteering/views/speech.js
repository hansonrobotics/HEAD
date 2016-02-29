define(['marionette', 'tpl!./templates/speech.tpl', 'lib/api'], function (Marionette, template, api) {
    return Marionette.LayoutView.extend({
        template: template,
        ui: {
            sayButton: '.app-say-button',
            speechInput: '.app-speech-input'
        },
        events: {
            'keyup @ui.speechInput': 'speechInputKeyPress',
            'click @ui.sayButton': 'saySpeech'
        },
        initialize: function (options) {
            console.log(options);
        },
        speechInputKeyPress: function (e) {
            if (e.keyCode == 13) this.saySpeech();
        },
        saySpeech: function () {
            var speech = this.ui.speechInput.val();
            if (speech) {
                var lang = this.options.interactionView ? this.options.interactionView.language : undefined;
                console.log(this.options.interactionView);
                this.ui.speechInput.val('');
                api.robotSpeech(speech, lang);
            }
        }
    });
});

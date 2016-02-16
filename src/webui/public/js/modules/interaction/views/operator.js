define(['marionette', 'tpl!./templates/operator.tpl', 'lib/api', 'jquery'], function (Marionette, template, api, $) {
    return Marionette.LayoutView.extend({
        template: template,
        ui: {
            sayButton: '.app-say-button',
            speechInput: '.app-speech-input',
            modeButtons: '.app-speech-mode-buttons .btn',
            suggestions: '.app-suggestions .label'
        },
        events: {
            'keyup @ui.speechInput': 'speechInputKeyPress',
            'click @ui.sayButton': 'saySpeech',
            'click @ui.modeButtons': 'changeMode',
            'click @ui.suggestions': 'suggestionSelected'
        },
        onShow: function () {
            // subscribe to suggestions; populate ui.suggestions el
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
        },
        changeMode: function (e) {
            var mode = $(e.target).data('mode');
            console.log(mode);
        },
        suggestionSelected: function (e) {
            var text = $(e.target).html();
            console.log(text);
        }
    });
});

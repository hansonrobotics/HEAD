define(['application', 'marionette', 'tpl!./templates/operator.tpl', 'lib/api', 'jquery'],
    function (app, Marionette, template, api, $) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                sayButton: '.app-say-button',
                speechInput: '.app-speech-input',
                modeButtons: '.app-speech-mode-buttons .btn',
                suggestionContainer: '.app-suggestions',
                suggestions: '.app-suggestions .label'
            },
            events: {
                'keyup @ui.speechInput': 'speechInputKeyPress',
                'click @ui.sayButton': 'saySpeech',
                'click @ui.modeButtons': 'changeMode',
                'click @ui.suggestions': 'suggestionSelected'
            },
            onShow: function () {
                var self = this;
                this.mode = 'automatic';

                api.disableTtsForwarding();
                api.topics.chatbot_responses['default'].subscribe(function (response) {
                    self.addSuggestion(response.data);
                });
                api.topics.chatbot_responses.en.subscribe(function (response) {
                    self.addSuggestion(response.data);
                });
                api.topics.chatbot_responses.zh.subscribe(function (response) {
                    self.addSuggestion(response.data);
                });
            },
            addSuggestion: function (msg) {
                if (this.mode == 'semi') {
                    this.ui.suggestionContainer.append($('<span>').addClass('label label-primary').html(msg));
                    this.bindUIElements();
                }
            },
            speechInputKeyPress: function (e) {
                if (e.keyCode == 13) this.saySpeech();
            },
            saySpeech: function () {
                var speech = this.ui.speechInput.val();
                if (speech) {
                    var lang = this.options.interactionView ? this.options.interactionView.language : undefined;
                    this.ui.speechInput.val('');
                    api.robotSpeech(speech, lang);
                }
            },
            changeMode: function (e) {
                this.ui.modeButtons.removeClass('active');
                this.mode = $(e.target).addClass('active').data('mode');

                if (this.mode == 'automatic') {
                    api.disableTtsForwarding();
                } else {
                    api.enableTtsForwarding();
                }

                if (this.mode != 'semi')
                    this.ui.suggestions.html('');
            },
            suggestionSelected: function (e) {
                var text = $(e.target).html();
                api.robotSpeech(text, app.language);
                this.ui.suggestions.html('');
            }
        });
    });

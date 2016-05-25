define(['application', 'marionette', 'tpl!./templates/operator.tpl', 'lib/api', 'jquery', 'underscore', 'typeahead'],
    function (app, Marionette, template, api, $, _) {
        return Marionette.LayoutView.extend({
            template: template,
            ui: {
                sayButton: '.app-say-button',
                speechInput: '.app-speech-input',
                modeButtons: '.app-speech-mode-buttons .btn',
                shortcutInfo: '.app-shortcut-info'
            },
            events: {
                'keyup @ui.speechInput': 'speechInputKeyPress',
                'click @ui.sayButton': 'sayClick',
                'click @ui.modeButtons': 'changeMode'
            },
            suggestions: {
                predefined: []
            },
            initialize: function (options) {
                this.mergeOptions(options, ['interactionView']);
            },
            onShow: function () {
                var self = this;
                this.ui.shortcutInfo.hide();
                this.mode = 'auto';
                this.ui.speechInput.typeahead({
                    items: 10,
                    source: function (query, process) {
                        process(self.suggestions.predefined);
                    },
                    updater: function (msg) {
                        self.saySpeech(msg);
                    }
                });

                api.getRosParam('/' + api.config.robot + '/webui/tts_suggestions', function (suggestions) {
                    if (Array.isArray(suggestions))
                        self.suggestions.predefined = suggestions;
                });
            },
            speechInputKeyPress: function (e) {
                if (e.keyCode == 13) this.sayClick();
            },
            sayClick: function () {
                var speech = this.ui.speechInput.val();
                this.saySpeech(speech);
            },
            saySpeech: function (speech) {
                if (speech) {
                    this.ui.speechInput.val('');
                    if (this.mode == 'auto')
                        api.robotSpeech(speech, app.language);
                    else
                        api.webSpeech(speech, app.language);
                }
            },
            changeMode: function (e) {
                this.ui.modeButtons.removeClass('active');
                this.mode = $(e.target).addClass('active').data('mode');

                if (this.mode == 'auto') {
                    api.disableTtsOperatorMode();
                } else {
                    api.enableTtsOperatorMode();
                }

                if (this.mode == 'semi')
                    this.ui.shortcutInfo.fadeIn();
                else
                    this.ui.shortcutInfo.fadeOut();
            }
        });
    });

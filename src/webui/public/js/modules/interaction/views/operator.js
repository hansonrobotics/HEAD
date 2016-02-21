define(['application', 'marionette', 'tpl!./templates/operator.tpl', 'lib/api', 'jquery', 'underscore', 'typeahead'],
    function (app, Marionette, template, api, $, _) {
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
                'click @ui.sayButton': 'sayClick',
                'click @ui.modeButtons': 'changeMode',
                'click @ui.suggestions': 'suggestionSelected'
            },
            suggestions: {
                predefined: [],
                responses: []
            },
            onShow: function () {
                var self = this;
                this.mode = 'auto';

                api.disableTtsForwarding();
                api.topics.chatbot_responses['default'].subscribe(function (response) {
                    self.clearSuggestions();
                    self.addSuggestion(response.data);
                });
                api.topics.chatbot_responses.en.subscribe(function (response) {
                    self.clearSuggestions();
                    self.addSuggestion(response.data);
                });
                api.topics.chatbot_responses.zh.subscribe(function (response) {
                    self.clearSuggestions();
                    self.addSuggestion(response.data);
                });

                this.initSuggestions();
            },
            initSuggestions: function () {
                var self = this;
                this.ui.speechInput.typeahead({
                    items: 10,
                    source: function (query, process) {
                        process(self.suggestions.predefined.concat(self.suggestions.responses));
                    },
                    updater: function (msg) {
                        self.saySpeech(msg);
                    }
                });

                api.getRosParam('/' + api.config.robot + '/webui/operator_suggestions', function (suggestions) {
                    if (Array.isArray(suggestions))
                        self.suggestions.predefined = suggestions;
                });

                this.setUpKeydownEvent();
            },
            setUpKeydownEvent: function () {
                var self = this,
                    selectResponse = function (e) {
                        var no = e.keyCode - 48;
                        if (self.isDestroyed)
                            $(window).off('keydown', selectResponse);
                        else if (no >= 0 && no <= 9 && !_.contains(['INPUT', 'TEXTAREA'], $(e.target).prop('tagName'))) {
                            if (self.suggestions.responses.length >= no) {
                                self.saySpeech(self.suggestions.responses[no - 1]);
                            }
                        }
                    };

                $(window).keydown(selectResponse);
            },
            clearSuggestions: function () {
                this.suggestions.responses = [];
                this.ui.suggestions.remove();
            },
            addSuggestion: function (msg) {
                if (this.mode == 'semi') {
                    this.suggestions.responses.push(msg);
                    this.ui.suggestionContainer.append($('<span>').addClass('label label-primary')
                        .html(this.suggestions.responses.length + '. ' + msg).data('msg', msg));
                    this.bindUIElements();
                }
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
                    this.clearSuggestions();
                    if (this.mode == 'auto')
                        api.robotSpeech(speech, app.language);
                    else
                        api.webSpeech(speech, app.language);
                }
            },
            changeMode: function (e) {
                this.clearSuggestions();
                this.ui.modeButtons.removeClass('active');
                this.mode = $(e.target).addClass('active').data('mode');

                if (this.mode == 'auto') {
                    api.disableTtsForwarding();
                } else {
                    api.enableTtsForwarding();
                }

                if (this.mode != 'semi')
                    this.ui.suggestions.html('');
            },
            suggestionSelected: function (e) {
                var text = $(e.target).data('msg');
                this.saySpeech(text);
            },
            keyDown: function (e) {
                console.log(e);
            }
        });
    });

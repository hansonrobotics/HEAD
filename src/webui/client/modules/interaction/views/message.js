define(["application", "./templates/message.tpl", 'lib/api', 'emoticons'], function (app, template, api) {
    app.module("Interaction.Views", function (Views, app, Backbone, Marionette, $, _) {
        Views.Message = Marionette.ItemView.extend({
            template: template,
            className: 'app-message',
            ui: {
                msg: '.msg',
                accept: '.app-accept',
                discard: '.app-discard',
                shortcutLabel: '.app-shortcut-label'
            },
            events: {
                'click @ui.accept': 'acceptSuggestion',
                'click @ui.discard': 'discardSuggestion'
            },
            modelEvents: {
                'change:hidden': 'hiddenChanged'
            },
            collectionEvents: {
                'add remove': 'updateFKeyLabel'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['interactionView']);
            },
            onAttach: function () {
                this.$el.addClass(this.model.get('author') == 'Robot' ? 'right' : 'left');
                this.ui.msg.emoticonize();
                this.updateFKeyLabel();
            },
            serializeData: function () {
                return _.extend(this.model.toJSON(), {
                    time: this.currentTime()
                });
            },
            hiddenChanged: function () {
                var self = this,
                    scroll = function () {
                        if (self.interactionView) self.interactionView.scrollToChatBottom();
                    };
                if (this.model.get('hidden'))
                    this.$el.fadeOut(null, scroll);
                else
                    this.$el.fadeIn(null, scroll);
            },
            updateFKeyLabel: function () {
                if (!this.isDestroyed) {
                    var type = this.model.get('type');
                    if (type && type == 'suggestion') {
                        var suggestions = this.collection.getSuggestions(),
                            length = suggestions.length,
                            i = suggestions.indexOf(this.model),
                            fKey = length - i;

                        this.ui.shortcutLabel.html('F' + fKey.toString()).stop();

                        if (i > -1 && fKey <= 12)
                            this.ui.shortcutLabel.fadeIn();
                        else
                            this.ui.shortcutLabel.fadeOut();
                    }
                }
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
            acceptSuggestion: function () {
                api.webSpeech(this.model.get('message'), app.language);
                this.discardSuggestion();
            },
            discardSuggestion: function () {
                this.collection.remove(this.model);
            }
        });
    });

    return app.module('Interaction.Views').Message;
});

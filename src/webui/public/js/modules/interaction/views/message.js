define(["application", "tpl!./templates/message.tpl", 'lib/api', 'emoticons'], function (app, template, api) {
    app.module("Interaction.Views", function (Views, app, Backbone, Marionette, $, _) {
        Views.Message = Marionette.ItemView.extend({
            template: template,
            className: 'app-message',
            ui: {
                msg: '.msg',
                accept: '.app-accept',
                discard: '.app-discard'
            },
            events: {
                'click @ui.accept': 'acceptSuggestion',
                'click @ui.discard': 'discardSuggestion'
            },
            initialize: function (options) {
                this.mergeOptions(options, ['collection']);
            },
            onRender: function () {
                this.$el.addClass(this.model.get('author') == 'Robot' ? 'right' : 'left');
                this.ui.msg.emoticonize();
            },
            serializeData: function () {
                return _.extend(this.model.toJSON(), {
                    time: this.currentTime()
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

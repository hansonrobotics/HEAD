define(["application", "tpl!./templates/emotion.tpl"], function (App, template) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Emotion = Marionette.ItemView.extend({
            template: template,
            tagName: 'span',
            className: 'app-emotion-slider-container',
            ui: {
                button: 'button'
            },
            events: {
                'click @ui.button': 'setEmotion'
            },
            setEmotion: function () {
                this.options.setEmotion(this.model.get('name'));

                var self = this;
                $(self.ui.button).addClass('active');

                setTimeout(function () {
                    $(self.ui.button).removeClass('active').blur();
                }, 2000)
            }
        });
    });

    return App.module('Gestures.Views').Emotion;
});

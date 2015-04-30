define(["application", "tpl!./templates/emotion.tpl", 'lib/api'], function (App, template, api) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Emotion = Marionette.ItemView.extend({
            template: template,
            tagName: 'li',
            className: 'app-emotion-slider-container',
            ui: {
                slider: '.app-slider',
                value: '.app-slider-value'
            },
            onRender: function () {
                var self = this;

                this.ui.slider.slider({
                    range: "min",
                    value: 0,
                    min: 0,
                    max: 100,
                    change: function (e, ui) {
                        self.model.set('value', ui.value / 100.0);
                        self.ui.value.html(ui.value);

                        self.resetTimeout = setTimeout(function () {
                            self.model.set('value', 0);
                            self.ui.value.html(0);
                            self.ui.slider.slider('value', 0);
                        }, 3000);
                    }
                });
            },
            onDestroy: function () {
                if (this.resetTimeout)
                    clearTimeout(this.resetTimeout);
            }
        });
    });

    return App.module('Gestures.Views').Emotion;
});

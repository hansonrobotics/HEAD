define(["application", "tpl!./templates/gesture.tpl", 'lib/api'], function (App, template, api) {
    App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Gesture = Marionette.ItemView.extend({
            tagName: 'span',
            template: template,
            ui: {
                button: 'button'
            },
            events: {
                'click @ui.button': 'gestureClicked'
            },
            gestureClicked: function () {
                var button = this.ui.button;
                $(button).addClass('active');

                api.setGesture(this.model.get('name'));

                setTimeout(function () {
                    $(button).removeClass('active');
                    $(button).blur();
                }, 2000)
            }
        });
    });

    return App.module('Gestures.Views').Gesture;
});

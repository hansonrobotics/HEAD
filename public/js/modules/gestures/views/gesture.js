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
                api.setGesture(this.model.get('name'));
                $(this.ui.button).addClass('active');

                var self = this;
                setTimeout(function () {
                    $(self.ui.button).removeClass('active');
                }, 2000)
            }
        });
    });

    return App.module('Gestures.Views').Gesture;
});

define(["application", "tpl!./templates/gesture.tpl"], function (App, template) {
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
                this.options.setGesture(this.model.get('name'));
                $(this.ui.button).addClass('active');

                var self = this;
                setTimeout(function () {
                    $(self.ui.button).removeClass('active').blur();
                }, 2000)
            }
        });
    });

    return App.module('Gestures.Views').Gesture;
});

define(["app", "tpl!./templates/motor.tpl"], function (UI, motorTpl) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Motor = Marionette.ItemView.extend({
            template: motorTpl
        });
    });

    return UI.Motors.View.Motor;
});

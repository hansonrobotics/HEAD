define(["app", "tpl!./templates/motor.tpl"], function (UI, motorTpl) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Motor = Marionette.ItemView.extend({
            template: motorTpl,
            ui: {
                slider: '.app-slider',
                value: '.app-slider-value'
            },
            onRender: function () {
                var self = this;
                $(this.ui.slider).slider({
                    range: "min",
                    value: this.model.getDefaultDeg(),
                    min: this.model.getMinDeg(),
                    max: this.model.getMaxDeg(),
                    slide: function (e, ui) {
                        self.model.setValueDeg(ui.value);
                        self.ui.value.text(ui.value);
                    },
                    start: function (e, ui) {
                        self.model.set('isActive', true);
                    },
                    stop: function (e, ui) {
                        self.model.set('isActive', false);
                    }
                });
            }
        });
    });

    return UI.Motors.View.Motor;
});

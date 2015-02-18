define(["app", "tpl!./templates/layout.tpl"], function (UI, layoutTpl) {
    UI.module("Motors.View", function (View, RosUI, Backbone, Marionette, $, _) {
        View.Layout = Marionette.LayoutView.extend({
            template: layoutTpl,

            regions: {
                motorRegion: "#app-motor-region"
            }
        });
    });

    return UI.Motors.View.Layout;
});

define(['ros_ui', 'tpl!./templates/layout.tpl'], function (RosUi, template) {
    RosUi.module("Motors.Common", function (Common, RosUI, Backbone, Marionette, $, _) {
        Common.Layout = Marionette.LayoutView.extend({
            template: template,

            regions: {
                motors: ".app-motors",
                expressions: ".app-expressions"
            }
        });
    });

    return RosUi.Motors.Common.Layout;
});
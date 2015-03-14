define(["application", "./frame", 'tpl!./templates/frames.tpl', 'jquery-ui'], function (App, frameView, framesTemplate) {
    App.module("Animations.Views", function (Views, App, Backbone, Marionette, $, _) {
        Views.Frames = Marionette.CompositeView.extend({
            template: framesTemplate,
            childViewContainer: '.app-frames',
            childView: frameView,
            ui: {
                container: '.app-frames'
            },
            events: {
                'click @ui.saveButton': 'updateAnimation'
            },
            initialize: function () {
                var self = this;
            },
            onRender: function () {
                var self = this;

                //$(this.ui.container).sortable({
                //    axis: "y",
                //    handle: ".app-motor-drag-handle",
                //    placeholder: "ui-state-highlight",
                //    deactivate: function () {
                //        $(self.ui.container).find('li').each(function (i, motor) {
                //            $(motor).attr('data-order-no', i);
                //            self.children.each(function (motorView) {
                //                if (motor == motorView.el) {
                //                    // updating order_no at relevant model
                //                    motorView.model.set('order_no', i);
                //                }
                //            });
                //        });
                //    }
                //});
            },
            updateAnimation: function () {
                this.collection.sync();
            }
        });
    });

    return App.Animations.Views.Frames;
});

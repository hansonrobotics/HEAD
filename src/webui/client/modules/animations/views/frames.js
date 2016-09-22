define(["application", "./frame", './templates/frames.tpl', 'jquery-ui'],
    function (App, frameView, framesTemplate, emptyTemplate) {
        App.module("Animations.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Frames = Marionette.CompositeView.extend({
                template: framesTemplate,
                childViewContainer: '.app-frames',
                childView: frameView,
                ui: {
                    container: '.app-frames'
                },
                onRender: function () {
                    var self = this;
                    this.setOrderNumbers();

                    $(this.ui.container).sortable({
                        axis: "y",
                        handle: ".app-frame-drag-handle",
                        placeholder: "ui-state-highlight",
                        deactivate: function () {
                            self.setOrderNumbers();
                        }
                    });
                },
                setOrderNumbers: function () {
                    var self = this;

                    $(this.ui.container).find('li').each(function (i, frame) {
                        self.children.each(function (motorView) {
                            if (frame == motorView.el) {
                                console.log('set');
                                // updating order_no at relevant model
                                motorView.model.set('order_no', i);
                            }
                        });
                    });
                }
            });
        });

        return App.Animations.Views.Frames;
    });

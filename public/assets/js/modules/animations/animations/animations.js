define(['application', 'tpl!./templates/animations.tpl', './frame', 'lib/api', './animations'],
    function (App, template, FrameView, api) {
        App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Animations = Marionette.ItemView.extend({
                template: template,
                collectionEvents: {
                    add: 'render'
                },
                ui: {
                    buttons: 'button'
                },
                events: {
                    'click @ui.buttons': 'buttonClicked'
                },
                serializeData: function () {
                    return {
                        animations: this.options.collection.toJSON()
                    };
                },
                buttonClicked: function (e) {
                    this.ui.buttons.removeClass('active');

                    var button = $(e.target);
                    button.addClass('active');

                    api.playAnimation(button.data('name'), 25);

                    // trigger event
                    Views.trigger('animation_selected', button);
                }
            });
        });

        return App.module('Animations.Views').Animations;
    });
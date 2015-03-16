define(['application', 'tpl!./templates/animations.tpl', './frame', 'lib/api', './animations'],
    function (App, template, FrameView, api) {
        App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Animations = Marionette.ItemView.extend({
                template: template,
                collectionEvents: {
                    add: 'render',
                    change: 'render'
                },
                ui: {
                    buttons: 'button'
                },
                events: {
                    'click @ui.buttons': 'buttonClicked'
                },
                initialize: function () {
                    this.selected_name = null;
                },
                serializeData: function () {
                    return {
                        animations: this.options.collection.toJSON(),
                        selected_name: this.selected_name
                    };
                },
                buttonClicked: function (e) {
                    this.ui.buttons.removeClass('active');
                    var name = $(e.target).addClass('active').data('name');
                    this.selected_name = name;

                    Views.trigger('animation_selected', name);
                    api.playAnimation(name, 25);
                }
            });
        });

        return App.module('Animations.Views').Animations;
    });
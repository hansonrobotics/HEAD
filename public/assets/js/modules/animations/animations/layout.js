define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Animations = Marionette.LayoutView.extend({
            template: template,
            ui: {
                enableEditButton: '.app-enable-edit',
                disableEditButton: '.app-disable-edit',
                motors: '.app-motors',
                frames: '.app-frames'
            },
            events: {
                'click @ui.enableEditButton': 'enableEditing',
                'click @ui.disableEditButton': 'disableEditing'
            },
            regions: {
                animationButtons: '.app-animations',
                motors: '.app-motors',
                frames: '.app-frames'
            },
            onRender: function () {
                this.ui.frames.hide();
                this.ui.motors.hide();
                this.disableEditing();
            },
            enableEditing: function () {
                this.ui.enableEditButton.addClass('active');
                this.ui.disableEditButton.removeClass('active');

                this.ui.motors.fadeIn();
            },
            disableEditing: function () {
                this.ui.disableEditButton.addClass('active');
                this.ui.enableEditButton.removeClass('active');

                this.ui.motors.fadeOut();
            }
        });
    });

    return App.module('Animations.Views').Animations;
});
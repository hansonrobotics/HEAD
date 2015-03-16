define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Animations = Marionette.LayoutView.extend({
            template: template,
            ui: {
                enableEditButton: '.app-enable-edit',
                disableEditButton: '.app-disable-edit',
                motors: '.app-motors',
                editing: '.app-editing',
                saveButton: '.app-save-frames'
            },
            events: {
                'click @ui.enableEditButton': 'enableEditing',
                'click @ui.disableEditButton': 'disableEditing',
                'click @ui.saveButton': 'updateAnimations'
            },
            regions: {
                animationButtons: '.app-animations',
                motors: '.app-motors',
                frames: '.app-frames',
                animationEdit: '.app-animation-edit'
            },
            onRender: function () {
                this.ui.editing.hide();
                this.disableEditing();
            },
            enableEditing: function () {
                this.ui.enableEditButton.addClass('active');
                this.ui.disableEditButton.removeClass('active');

                this.ui.editing.fadeIn();
            },
            disableEditing: function () {
                this.ui.disableEditButton.addClass('active');
                this.ui.enableEditButton.removeClass('active');

                this.ui.editing.fadeOut();
            },
            updateAnimations: function () {
                this.options.animationsCollection.sync();
            }
        });
    });

    return App.module('Animations.Views').Animations;
});
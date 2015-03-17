define(['application', 'tpl!./templates/layout.tpl'], function (App, template) {
    App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
        Views.Animations = Marionette.LayoutView.extend({
            template: template,
            ui: {
                enableEditButton: '.app-enable-edit',
                disableEditButton: '.app-disable-edit',
                motors: '.app-motors',
                editing: '.app-editing',
                saveButton: '.app-save-frames',
                addFrame: '.app-add-frame',
                deleteAnimation: '.app-delete-animation',
                addAnimation: '.app-add-animation',
                adminUI: '.app-admin'
            },
            events: {
                'click @ui.enableEditButton': 'enableEditing',
                'click @ui.disableEditButton': 'disableEditing',
                'click @ui.saveButton': 'updateAnimations',
                'click @ui.addFrame': 'addFrame',
                'click @ui.deleteAnimation': 'deleteAnimation',
                'click @ui.addAnimation': 'addAnimation'
            },
            regions: {
                animationButtons: '.app-animations',
                motors: '.app-motors',
                frames: '.app-frames',
                animationEdit: '.app-animation-edit'
            },
            onRender: function () {
                this.ui.editing.hide();
                this.ui.adminUI.hide();
                this.disableEditing();

                var self = this;
                App.getAdminEnabled(function (enabled) {
                    if (enabled) self.ui.adminUI.show();
                });
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
            },
            addFrame: function () {
                Views.trigger('add_frame');
            },
            deleteAnimation: function () {
                Views.trigger('delete_animation');
            },
            addAnimation: function () {
                Views.trigger('add_animation');
            }
        });
    });

    return App.module('Animations.Views').Animations;
});
define(['application', 'tpl!./templates/layout.tpl', 'bootstrap'],
    function (App, template) {
        App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Animations = Marionette.LayoutView.extend({
                template: template,
                ui: {
                    motors: '.app-motors',
                    saveButton: '.app-save-frames',
                    addFrame: '.app-add-frame',
                    deleteAnimation: '.app-delete-animation',
                    addAnimation: '.app-add-animation',
                    admin: '.app-admin'
                },
                events: {
                    'click @ui.saveButton': 'updateAnimations',
                    'click @ui.addFrame': 'addFrame',
                    'click @ui.deleteAnimation': 'deleteAnimation',
                    'click @ui.addAnimation': 'addAnimation'
                },
                regions: {
                    animationButtons: '.app-animations',
                    animationEdit: '.app-animation-edit',
                    frames: '.app-frames',
                    motors: '.app-motors'
                },
                updateAnimations: function () {
                    var self = this;

                    this.options.animationsCollection.sync(function () {
                        App.Utilities.showPopover(self.ui.saveButton, 'Saved')
                    }, function () {
                        App.Utilities.showPopover(self.ui.saveButton, 'Error saving animations')
                    });
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
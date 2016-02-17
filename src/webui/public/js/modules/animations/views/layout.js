define(['application', 'tpl!./templates/layout.tpl', 'lib/api', 'bootstrap'],
    function (App, template, api) {
        App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Animations = Marionette.LayoutView.extend({
                template: template,
                ui: {
                    motors: '.app-motors',
                    saveButton: '.app-save-frames',
                    addFrame: '.app-add-frame',
                    deleteAnimation: '.app-delete-animation',
                    addAnimation: '.app-add-animation',
                    enableTorque: '.app-enable-torque',
                    disableTorque: '.app-disable-torque',
                    readValues: '.app-read-values'
                },
                events: {
                    'click @ui.saveButton': 'updateAnimations',
                    'click @ui.addFrame': 'addFrame',
                    'click @ui.deleteAnimation': 'deleteAnimation',
                    'click @ui.addAnimation': 'addAnimation',
                    'click @ui.enableTorque': 'enableTorque'
                    'click @ui.disableTorque': 'disableTorque'
                    'click @ui.readValues': 'readValues'
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
                },
                enableTorque: function () {
                    api.setDxlTorque(true);
                },
                disableTorque: function () {
                    api.setDxlTorque(false);
                },
                readValues: function () {
                },

            });
        });

        return App.module('Animations.Views').Animations;
    });
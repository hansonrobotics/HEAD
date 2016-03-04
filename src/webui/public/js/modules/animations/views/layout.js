define(['application', 'tpl!./templates/layout.tpl', 'lib/api', 'bootstrap', 'entities/expression', 'scrollbar'],
    function (App, template, api) {
        App.module('Animations.Views', function (Views, App, Backbone, Marionette, $, _) {
            Views.Animations = Marionette.LayoutView.extend({
                template: template,
                ui: {
                    motors: '.app-motors',
                    saveButton: '.app-save-frames',
                    addFrame: '.app-add-frame',
                    deleteAnimation: '.app-delete-animation',
                    copyAnimation: '.app-copy-animation',
                    addAnimation: '.app-add-animation',
                    admin: 'app-admin',
                    enableTorque: '.app-enable-torque',
                    disableTorque: '.app-disable-torque',
                    readValues: '.app-read-values',
                    expressions: '.app-expressions',
                    animationsColumn: '.app-animations-column',
                    motorsColumn: '.app-motors-column'
                },
                events: {
                    'click @ui.saveButton': 'updateAnimations',
                    'click @ui.addFrame': 'addFrame',
                    'click @ui.copyAnimation': 'copyAnimation',
                    'click @ui.deleteAnimation': 'copyAnimation',
                    'click @ui.addAnimation': 'addAnimation',
                    'click @ui.enableTorque': 'enableTorque',
                    'click @ui.disableTorque': 'disableTorque',
                    'click @ui.readValues': 'readValues'
                },
                regions: {
                    animationButtons: '.app-animations',
                    animationEdit: '.app-animation-edit',
                    frames: '.app-frames-container',
                    motors: '.app-motors'
                },
                onRender: function () {
                    var self = this,
                        expressions = new App.Entities.ExpressionCollection();
                    expressions.on('add', function (model) {
                        self.ui.expressions.append($('<button>').addClass('expression-button btn btn-default').html(
                            model.get('name')).click(function () {
                            Views.trigger('add_expression_frame', model);
                        }));
                    });
                    expressions.fetch();

                    var resizeColumns = function () {
                        if (self.isDestroyed)
                            $(window).off('resize', resizeColumns);
                        else {
                            var height = App.LayoutInstance.getContentHeight();
                            $(self.ui.animationsColumn).height(height);
                            $(self.ui.motorsColumn).height(height);
                            self.ui.animationsColumn.perfectScrollbar('update');
                            self.ui.motorsColumn.perfectScrollbar('update');
                        }
                    };

                    this.ui.animationsColumn.perfectScrollbar();
                    this.ui.motorsColumn.perfectScrollbar();

                    $(window).resize(resizeColumns).resize();
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
                copyAnimation: function () {
                    Views.trigger('copy_animation');
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
                    api.getMotorStates(function (val) {
                        Views.trigger('read_values', val);
                    })
                }
            });
        });

        return App.module('Animations.Views').Animations;
    });
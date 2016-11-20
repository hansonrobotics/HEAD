define(['application', './templates/layout.tpl', 'lib/api', 'modules/motors/views/motors', './frames',
        './animation_edit', './animations', 'entities/expression_collection', 'bootstrap', 'scrollbar', 'scrollbar-css'],
    function (App, template, api, MotorsView, FramesView, AnimationEditView, AnimationsView, ExpressionCollection) {
        return Marionette.View.extend({
            template: template,
            ui: {
                motors: '.app-motors',
                saveButton: '.app-save-frames',
                addFrame: '.app-add-frame',
                deleteAnimation: '.app-delete-animation',
                copyAnimation: '.app-copy-animation',
                addAnimation: '.app-add-animation',
                enableTorque: '.app-enable-torque',
                disableTorque: '.app-disable-torque',
                updateMotorState: '.app-read-values',
                expressions: '.app-expressions',
                animationsColumn: '.app-animations-column',
                motorsColumn: '.app-motors-column'
            },
            events: {
                'click @ui.saveButton': 'updateAnimations',
                'click @ui.addFrame': 'addFrameClick',
                'click @ui.copyAnimation': 'copyAnimation',
                'click @ui.deleteAnimation': 'deleteAnimation',
                'click @ui.addAnimation': 'addAnimation',
                'click @ui.enableTorque': 'enableTorque',
                'click @ui.disableTorque': 'disableTorque',
                'click @ui.readValues': 'updateMotorState'
            },
            regions: {
                animationButtons: '.app-animations',
                animationEdit: '.app-animation-edit',
                frames: '.app-frames-container',
                motors: '.app-motors'
            },
            onRender: function () {
                this.showAnimations();
                var self = this,
                    expressions = new ExpressionCollection();
                expressions.on('add', function (model) {
                    self.ui.expressions.append($('<button>').addClass('expression-button btn btn-default').html(
                        model.get('name')).click(function () {
                        Views.trigger('add_expression_frame', model);
                    }));
                });
                expressions.fetch();

                var resizeColumns = function () {
                    if (self.isDestroyed())
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
                this.showMotors();
                $(window).resize(resizeColumns).resize();

                App.module('Animations.Views').on('frame_selected', function (frame) {
                    self.frameSelected(frame);
                }).on('animation_selected', function (name) {
                    self.animationSelected(name);
                }).on('add_frame', function () {
                    self.addFrame('', {});
                }).on('copy_frame', function (frame) {
                    self.copyFrame(frame);
                }).on('add_expression_frame', function (expression) {
                    self.addFrame(expression.get('name'), expression.get('motor_positions'));
                });
            },
            showAnimations: function () {
                this.animationsCollection = new App.Entities.AnimationsCollection();
                this.animationsView = new AnimationsView({collection: this.animationsCollection});
                this.getRegion('animationButtons').show(this.animationsView);

                // load data
                this.animationsCollection.fetch();
            },
            showMotors: function () {
                var self = this;

                this.motorsCollection = new App.Entities.MotorCollection();
                this.motorsCollection.fetchFromParam(function () {
                    self.motorsCollection.setDefaultValues();
                });
                this.motorsView = new MotorsView({collection: this.motorsCollection, monitoring: false});
                this.getRegion('motors').show(this.motorsView);

                self.motorsCollection.on('change', function () {
                    self.updateFrame();
                });
            },
            updateFrame: function () {
                if (typeof this.selected_frame != 'undefined' && this.selected_frame)
                    this.selected_frame.set('motors', this.motorsCollection.getRelativePositions());
            },
            updateAnimations: function () {
                var self = this;

                this.animationsCollection.sync(function () {
                    App.Utilities.showPopover(self.ui.saveButton, 'Saved')
                }, function () {
                    App.Utilities.showPopover(self.ui.saveButton, 'Error saving animations')
                });
            },
            enableTorque: function () {
                api.setDxlTorque(true);
            },
            disableTorque: function () {
                api.setDxlTorque(false);
            },
            updateMotorState: function () {
                this.motorsCollection.fetchStates();
            },
            animationSelected: function (animation) {
                if (typeof this.last_animation == 'undefined' || this.last_animation != animation) {
                    this.last_animation = animation;
                    this.selected_frame = null;

                    var framesView = new FramesView({collection: animation.get('frames_collection')});
                    this.getRegion('frames').show(framesView);

                    var animationEditView = new AnimationEditView({model: animation});
                    this.getRegion('animationEdit').show(animationEditView);
                }
            },
            frameSelected: function (frame) {
                if (typeof this.selected_frame == 'undefined' || this.selected_frame != frame) {
                    // reset so that previous frame is not touched during new frame initialization
                    this.selected_frame = null;

                    this.motorsCollection.each(function (motor) {
                        var motorPosition = frame.get('motors')[motor.get('name')];

                        if (typeof motorPosition == 'undefined') {
                            motor.selected(false);
                            motor.set('value', motor.get('default'));
                        } else {
                            motor.selected(true);
                            motor.setRelativeVal('value', motorPosition);
                        }
                    });

                    this.motorsView.showSelectButtons(true);
                    this.selected_frame = frame;
                }
            },
            addFrameClick: function () {
                this.addFrame('', {});
            },
            addFrame: function (name, positions) {
                if (typeof this.last_animation != 'undefined') {
                    var frames = this.last_animation.get('frames_collection');
                    frames.add(new Backbone.Model({
                        acceleration: 0,
                        frames: 1,
                        motors: positions,
                        name: name,
                        speed: 0
                    }));
                }
            },
            deleteAnimation: function () {
                if (typeof this.last_animation != 'undefined'
                    && confirm("Are you sure you want to delete currently selected animation?")) {

                    this.getRegion('frames').reset();
                    this.getRegion('animationEdit').reset();
                    this.last_animation.destroy();
                }
            },
            copyAnimation: function () {
                if (typeof this.last_animation != 'undefined')
                    this.animationsCollection.add(new App.Entities.Animation({
                        name: this.last_animation.get('name') + '_copy',
                        frames_collection: this.last_animation.get('frames_collection').clone()
                    }));
            },
            addAnimation: function () {
                this.animationsCollection.add(new App.Entities.Animation({
                    name: 'New_Animation',
                    frames_collection: new App.Entities.FramesCollection()
                }));
            },
            copyFrame: function (frame) {
                if (typeof this.last_animation != 'undefined') {
                    var frames = this.last_animation.get('frames_collection');
                    var clone = frame.clone();
                    clone.unset('order_no');
                    clone.set('name', clone.get('name') + '_Copy');

                    frames.add(clone);
                }
            }
        });
    });

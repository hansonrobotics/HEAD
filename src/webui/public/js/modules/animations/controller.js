define(['application', './views/animations', './views/layout', '../motors/views/motors',
        './views/frames', './views/animation_edit', 'lib/api', 'entities/animation', 'entities/motor'],
    function (App, AnimationsView, LayoutView, MotorsView, FramesView, AnimationEditView, api) {
        return {
            index: function () {
                api.disableInteractionMode();
                api.blenderMode.disable();
                api.setExpression("Neutral", 0);
                api.pointHead();


                this.animationsCollection = new App.Entities.AnimationsCollection();
                this.layoutView = new LayoutView({animationsCollection: this.animationsCollection});
                this.animationsView = new AnimationsView({collection: this.animationsCollection});

                // show views
                App.LayoutInstance.getRegion('content').show(this.layoutView);
                this.layoutView.ui.admin.hide();

                App.LayoutInstance.setTitle('Animations');
                App.LayoutInstance.showNav();

                // show animations
                this.layoutView.getRegion('animationButtons').show(this.animationsView);

                // load data
                this.animationsCollection.fetch();
            },
            admin_index: function () {
                var self = this;

                // init index
                this.index();
                this.layoutView.ui.admin.show();

                App.LayoutInstance.showAdminNav();

                this.motorsCollection = new App.Entities.MotorCollection();
                this.motorsCollection.fetchFromParam();

                this.motorsView = new MotorsView({collection: this.motorsCollection});
                this.layoutView.getRegion('motors').show(this.motorsView);

                App.module('Animations.Views').on('frame_selected', function (frame) {
                    self.frameSelected(frame);
                }).on('animation_selected', function (name) {
                    self.animationSelected(name);
                }).on('add_frame', function () {
                    self.addFrame();
                }).on('delete_animation', function () {
                    self.deleteAnimation();
                }).on('add_animation', function () {
                    self.addAnimation();
                }).on('copy_frame', function (frame) {
                    self.copyFrame(frame);
                });

                self.motorsCollection.on('change', function () {
                    self.updateFrame();
                });
            },
            animationSelected: function (animation) {
                if (typeof this.last_animation == 'undefined' || this.last_animation != animation) {
                    this.last_animation = animation;
                    this.selected_frame = null;

                    var framesView = new FramesView({collection: animation.get('frames_collection')});
                    this.layoutView.getRegion('frames').show(framesView);

                    var animationEditView = new AnimationEditView({model: animation});
                    this.layoutView.getRegion('animationEdit').show(animationEditView);
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
            updateFrame: function () {
                if (typeof this.selected_frame != 'undefined' && this.selected_frame)
                    this.selected_frame.set('motors', this.motorsCollection.getRelativePositions());
            },
            addFrame: function () {
                if (typeof this.last_animation != 'undefined') {
                    var frames = this.last_animation.get('frames_collection');
                    frames.add(new Backbone.Model({
                        acceleration: 0,
                        frames: 1,
                        motors: {},
                        name: '',
                        speed: 0
                    }));
                }
            },
            deleteAnimation: function () {
                if (typeof this.last_animation != 'undefined'
                    && confirm("Are you sure you want to delete currently selected animation?")) {

                    this.layoutView.getRegion('frames').reset();
                    this.layoutView.getRegion('animationEdit').reset();
                    this.last_animation.destroy();
                }
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
                    clone.set('name', clone.get('name') + '_Copy')

                    frames.add(clone);
                }
            }
        }
    });

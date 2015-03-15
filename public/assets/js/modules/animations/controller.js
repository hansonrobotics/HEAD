define(['jquery', 'application', './views/animations', './views/layout', '../motor/show/motors',
        './views/frames', 'lib/api', 'entities/animation'],
    function ($, App, AnimationsView, LayoutView, MotorsView, FramesView, api) {
        return {
            init: function () {
                var self = this;

                this.animationsCollection = new App.Entities.AnimationsCollection();
                this.motorsCollection = new App.Entities.MotorCollection();

                this.layoutView = new LayoutView({animationsCollection: this.animationsCollection});
                this.motorsView = new MotorsView({collection: this.motorsCollection, disable_edit: true});
                this.animationsView = new AnimationsView({collection: this.animationsCollection});

                // show views
                $('#app-page-animations').html(this.layoutView.render().el);
                this.layoutView.getRegion('animationButtons').show(this.animationsView);
                this.layoutView.getRegion('motors').show(this.motorsView);

                // load data
                this.animationsCollection.fetch();
                api.getMotorsFromFile(function (data) {
                    self.motorsCollection.add(data);
                });

                App.module('Animations.Views').on('frame_selected', function (frame) {
                    self.frameSelected(frame);
                });

                App.module('Animations.Views').on('animation_selected', function (name) {
                    self.animationSelected(name);
                });

                this.motorsCollection.on('change', function () {
                    self.updateFrame();
                });
            },
            animationSelected: function (name) {
                if (typeof this.last_animation == 'undefined' || this.last_animation != name) {
                    this.last_animation = name;
                    this.selected_frame = null;

                    // find animation model
                    var animation = this.animationsCollection.find(function (model) {
                        return model.get('name') == name;
                    });

                    var framesView = new FramesView({
                        collection: animation.get('frames_collection')
                    });

                    this.layoutView.getRegion('frames').show(framesView);
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
            }
        }
    });

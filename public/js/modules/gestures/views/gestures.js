define(["application", "./gesture", 'tpl!./templates/gestures.tpl', 'lib/api'],
    function (App, GestureView, template, api) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Gestures = Marionette.CompositeView.extend({
                childView: GestureView,
                template: template,
                childItemContainer: '.app-gestures',
                ui: {
                    speedSlider: '.app-speed-slider',
                    durationValue: '.app-speed-value',
                    magnitudeSlider: '.app-magnitude-slider',
                    magnitudeValue: '.app-magnitude-value'
                },
                config: {
                    speed: 0.5,
                    magnitude: 0.5
                },
                childViewOptions: function () {
                    var self = this;

                    // pass child views a method for changing emotions
                    return {
                        setGesture: function (name) {
                            api.setGesture(name, 1, self.config.speed, self.config.magnitude)
                        }
                    };
                },
                serializeData: function () {
                    return this.config;
                },
                onRender: function () {
                    var self = this;

                    // init speed slider
                    this.ui.speedSlider.slider({
                        range: "min",
                        min: 0,
                        max: 1000,
                        value: this.config.speed * 100,
                        change: function (e, ui) {
                            var duration = ui.value / 100.0;

                            // update ui label
                            self.ui.durationValue.html(duration);

                            // update config
                            self.config.speed = duration;
                        }
                    });

                    // init magnitude slider
                    this.ui.magnitudeSlider.slider({
                        range: "min",
                        min: 0,
                        max: 100,
                        value: this.config.magnitude * 100,
                        change: function (e, ui) {
                            // update ui label
                            self.ui.magnitudeValue.html(ui.value);

                            // update config
                            self.config.magnitude = ui.value / 100.0;
                        }
                    });
                }
            });
        });

        return App.module('Gestures.Views').Gestures;
    });

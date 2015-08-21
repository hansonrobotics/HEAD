define(["application", "./gesture", 'tpl!./templates/gestures.tpl'],
    function (App, GestureView, template) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Gestures = Marionette.CompositeView.extend({
                childView: GestureView,
                template: template,
                childItemContainer: '.app-gestures',
                ui: {
                    speedSlider: '.app-speed-slider',
                    speedValue: '.app-speed-value',
                    magnitudeSlider: '.app-magnitude-slider',
                    magnitudeValue: '.app-magnitude-value'
                },
                config: {
                    speed: {
                        default: 1,
                        current: 1,
                        min: 0,
                        max: 10
                    },
                    magnitude: {
                        default: 1,
                        current: 1,
                        min: 0,
                        max: 1
                    }
                },
                childViewOptions: function () {
                    return {
                        config: this.config,
                        getSliderValues: function () {
                            return {
                                speed: this.config.speed.current.toFixed(2),
                                magnitude: this.config.magnitude.current.toFixed(2)
                            }
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
                        min: this.config.speed.min * 100,
                        max: this.config.speed.max * 100,
                        value: this.config.speed.current * 100,
                        change: function (e, ui) {
                            var speed = ui.value / 100.0;

                            // update ui label
                            self.ui.speedValue.html(speed);

                            // update config
                            self.config.speed.current = speed;
                        }
                    });

                    // init magnitude slider
                    this.ui.magnitudeSlider.slider({
                        range: "min",
                        min: this.config.magnitude.min * 100,
                        max: this.config.magnitude.max * 100,
                        value: this.config.magnitude.current * 100,
                        change: function (e, ui) {
                            // update ui label
                            self.ui.magnitudeValue.html(ui.value);

                            // update config
                            self.config.magnitude.current = ui.value / 100.0;
                        }
                    });
                }
            });
        });

        return App.module('Gestures.Views').Gestures;
    });

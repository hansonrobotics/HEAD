define(["application", "./emotion", 'tpl!./templates/emotions.tpl', 'lib/api'],
    function (App, EmotionView, template, api) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Emotions = Marionette.CompositeView.extend({
                childView: EmotionView,
                childViewContainer: '.app-emotions-container',
                template: template,
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
                        setEmotion: function (name) {
                            api.setEmotion(name, self.config.magnitude, self.config.speed)
                        }
                    };
                },
                /**
                 * Pass data to the template
                 *
                 * @returns {Views.Emotions.config|{speed, magnitude}}
                 */
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
                            var speed = ui.value / 100.0;

                            // update ui label
                            self.ui.durationValue.html(speed);

                            // update config
                            self.config.speed = speed;
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

        return App.module('Gestures.Views').Emotions;
    });

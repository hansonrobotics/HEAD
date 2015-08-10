define(["application", "tpl!./templates/emotion.tpl", 'lib/api', 'lib/behaviors/touch_button'],
    function (App, template, api) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Emotion = Marionette.ItemView.extend({
                template: template,
                tagName: 'span',
                className: 'app-emotion-slider-container',
                ui: {
                    button: 'button',
                    duration: '.app-duration',
                    intensity: '.app-intensity'
                },
                behaviors: {
                    TouchButton: {
                    }
                },
                events: {
                    'click @ui.button': 'onClick'
                },
                /**
                 * Store config, hook on to TouchButton behavior events
                 * @param options
                 */
                initialize: function (options) {
                    // recursive object clone
                    this.config = JSON.parse(JSON.stringify(options.config));

                    // set current parameter values to default
                    this.config.duration.current = this.config.duration.default;
                    this.config.magnitude.current = this.config.magnitude.default;

                    this.on('touch_button:start', this.touchStart);
                    this.on('touch_button:change', this.touchMove);
                    this.on('touch_button:end', this.setEmotion);
                },
                /**
                 * Handles regular click event on a desktop
                 */
                onClick: function () {
                    if (!this.touch) {
                        var config = this.options.getSliderValues();
                        this.setEmotion(config.duration, config.magnitude);
                    }

                    this.touch = false;
                },
                /**
                 * Handle TouchButton start event
                 */
                touchStart: function () {
                    this.touch = true;

                    this.showIndicators();
                    this.updateIndicators();
                },
                /**
                 * Handle TouchButton change event
                 */
                touchMove: function (diff) {
                    // update duration
                    this.config.duration.current = Math.min(this.config.duration.max,
                        Math.max(this.config.duration.min, this.config.duration.current + diff.x));

                    // update magnitude
                    this.config.magnitude.current = Math.min(this.config.magnitude.max,
                        Math.max(this.config.magnitude.min, this.config.magnitude.current + diff.y));

                    this.updateIndicators();
                },
                setEmotion: function (duration, magnitude) {
                    if (typeof duration == 'undefined')
                        duration = this.config.duration.current.toFixed(2);

                    if (typeof magnitude == 'undefined')
                        magnitude = this.config.magnitude.current.toFixed(2);

                    if (duration > 0) {
                        api.setEmotion(this.model.get('name'), magnitude, duration);

                        // hide indicators after duration
                        var self = this;
                        setTimeout(function () {
                            self.hideIndicators();
                        }, duration * 1000)
                    } else {
                        this.hideIndicators();
                    }
                },
                /**
                 * Pass data to the template
                 */
                serializeData: function () {
                    return _.extend(this.model.toJSON(), this.config);
                },
                /**
                 * Hide indicators initially
                 */
                onRender: function () {
                    this.hideIndicators();
                },
                updateIndicators: function () {
                    this.ui.duration.html(this.config.duration.current.toFixed(2) + 's');
                    this.ui.intensity.html(this.config.magnitude.current.toFixed(2) * 100 + '%');
                },
                hideIndicators: function () {
                    $(this.ui.button).removeClass('active').blur();

                    this.ui.duration.hide();
                    this.ui.intensity.hide();
                },
                showIndicators: function () {
                    this.ui.button.addClass('active');

                    this.ui.duration.fadeIn();
                    this.ui.intensity.fadeIn();
                }
            });
        });

        return App.module('Gestures.Views').Emotion;
    });

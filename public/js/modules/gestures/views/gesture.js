define(["application", "tpl!./templates/gesture.tpl", 'lib/api', 'lib/behaviors/touch_button'],
    function (App, template, api) {
        App.module("Gestures.Views", function (Views, App, Backbone, Marionette, $, _) {
            Views.Gesture = Marionette.ItemView.extend({
                tagName: 'span',
                template: template,
                ui: {
                    button: 'button',
                    duration: '.app-duration',
                    intensity: '.app-intensity'
                },
                behaviors: {
                    TouchButton: {}
                },
                events: {
                    'click @ui.button': 'onClick'
                },
                /**
                 * Store config, hook on to TouchButton behavior events
                 */
                initialize: function (options) {
                    // recursive object clone
                    this.config = JSON.parse(JSON.stringify(options.config));

                    // set current parameter values to default
                    this.config.speed.current = this.config.speed.default;
                    this.config.magnitude.current = this.config.magnitude.default;

                    this.on('touch_button:start', this.touchStart);
                    this.on('touch_button:change', this.touchMove);
                    this.on('touch_button:end', this.setGesture);
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
                    this.config.speed.current = Math.min(this.config.speed.max,
                        Math.max(this.config.speed.min, this.config.speed.current + diff.x));

                    // update magnitude
                    this.config.magnitude.current = Math.min(this.config.magnitude.max,
                        Math.max(this.config.magnitude.min, this.config.magnitude.current + diff.y));

                    this.updateIndicators();
                },
                onClick: function () {
                    if (!this.touch) {
                        var config = this.options.getSliderValues();
                        this.setGesture(config.speed, config.magnitude);
                    }

                    this.touch = false;
                },
                /**
                 * Pass data to the template
                 */
                serializeData: function () {
                    return _.extend(this.model.toJSON(), this.options.config);
                },
                /**
                 * Hide indicators initially
                 */
                onRender: function () {
                    this.hideIndicators();
                },
                updateIndicators: function () {
                    this.ui.duration.html(this.config.speed.current.toFixed(2) + 's');
                    this.ui.intensity.html((this.config.magnitude.current * 100).toFixed(2) + '%');
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
                },
                setGesture: function (speed, magnitude) {
                    if (typeof speed == 'undefined')
                        speed = this.config.speed.current.toFixed(2);

                    if (typeof magnitude == 'undefined')
                        magnitude = this.config.magnitude.current.toFixed(2);

                    if (speed > 0) {
                        api.setGesture(this.model.get('name'), 1, speed, magnitude);

                        // hide indicators after duration
                        var self = this;
                        setTimeout(function () {
                            self.hideIndicators();
                        }, speed * 1000)
                    } else {
                        this.hideIndicators();
                    }
                }
            });
        });

        return App.module('Gestures.Views').Gesture;
    });

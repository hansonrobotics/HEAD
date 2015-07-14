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
                    TouchButton: {}
                },
                events: {
                    'click @ui.button': 'onClick'
                },
                /**
                 * Handles regular click event on a desktop
                 */
                onClick: function () {
                    var duration = this.config.speed.current,
                        self = this;

                    if (duration > 0) {
                        api.setGesture(this.model.get('name'), 1, duration, this.config.magnitude.current);
                        setTimeout(function () {
                            self.hideIndicators();
                        }, duration * 1000)
                    }
                },
                /**
                 * Store config, hook on to TouchButton behavior events
                 * @param options
                 */
                initialize: function (options) {
                    this.config = options.config;

                    this.on('touch_button:start', this.touchStart);
                    this.on('touch_button:change', this.touchMove);
                    this.on('touch_button:end', this.touchEnd);
                },
                /**
                 * Handle TouchButton start event
                 */
                touchStart: function () {
                    this.showIndicators();

                    // set current parameter values to default
                    this.config.speed.current = this.config.speed.default;
                    this.config.magnitude.current = this.config.magnitude.default;

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
                /**
                 * Handle TouchButton end event
                 */
                touchEnd: function () {
                    var duration = this.config.speed.current.toFixed(2);

                    if (duration > 0) {
                        api.setGesture(this.model.get('name'), 1, duration, this.config.magnitude.current);

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
                }
            });
        });

        return App.module('Gestures.Views').Emotion;
    });

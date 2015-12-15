define(['application', './init'], function (App) {
    App.module('Behaviors', function (Behaviors, App, Backbone, Marionette, $, _) {
        /**
         * Tracks touch event on a button
         */
        Behaviors.TouchButton = Marionette.Behavior.extend({
            defaults: {},
            events: {
                'touchstart @ui.button': 'touchStart',
                'touchmove @ui.button': 'touchMove',
                'touchend @ui.button': 'touchEnd'
            },
            touchStart: function () {
                this.view.trigger('touch_button:start');
            },
            touchMove: function (e) {
                e.preventDefault();

                // get current coordinates
                var current = {
                    x: e.originalEvent.touches[0].clientX,
                    y: e.originalEvent.touches[0].clientY
                };

                // if not the first
                if (this.lastTouch) {
                    var diff = {
                        x: (current.x - this.lastTouch.x) / 10,
                        y: -(current.y - this.lastTouch.y) / 100
                    };

                    // trigger change event with axis' diff as a parameter
                    this.view.trigger('touch_button:change', diff);
                }

                // store current coordinates
                this.lastTouch = current;
            },
            touchEnd: function () {
                this.view.trigger('touch_button:end');
                delete this.lastTouch;
            }
        });
    });
});
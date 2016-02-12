define(["marionette", "tpl!./templates/gesture.tpl", 'lib/api', 'lib/behaviors/touch_button'],
    function (Marionette, template, api) {
        return Marionette.ItemView.extend({
            tagName: 'span',
            template: template,
            ui: {
                button: 'button',
                speedBar: '.app-speed-bar',
                magnitudeBar: '.app-magnitude-bar',
                speedIndicator: '.app-speed-indicator',
                magnitudeIndicator: '.app-magnitude-indicator'
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
                this.ui.speedIndicator.html(this.config.speed.current.toFixed(2) + 's');
                this.ui.magnitudeIndicator.html((this.config.magnitude.current * 100).toFixed(2) + '%');

                var speed = (this.config.speed.current - this.config.speed.min) /
                    (this.config.speed.max - this.config.speed.min);

                // update speed and magnitude bars
                this.ui.speedBar.css('width', parseInt(speed * 100) + '%');
                this.ui.magnitudeBar.css('height', parseInt(this.config.magnitude.current * 100) + '%');

            },
            hideIndicators: function () {
                $(this.ui.button).removeClass('active').blur();

                this.ui.speedIndicator.hide();
                this.ui.magnitudeIndicator.hide();
            },
            showIndicators: function () {
                this.ui.button.addClass('active');

                this.ui.speedIndicator.fadeIn();
                this.ui.magnitudeIndicator.fadeIn();
            },
            setGesture: function (speed, magnitude) {
                if (typeof speed == 'undefined')
                    speed = this.config.speed.current.toFixed(2);

                if (typeof magnitude == 'undefined')
                    magnitude = this.config.magnitude.current.toFixed(2);

                if (speed > 0) {
                    api.setGesture(this.model.get('name'), 1, parseFloat(speed), parseFloat(magnitude));

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

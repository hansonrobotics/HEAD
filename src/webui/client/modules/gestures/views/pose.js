define(["marionette", "./templates/pose.tpl", 'lib/api'],
    function (Marionette, template, api) {
        return Marionette.ItemView.extend({
            template: template,
            tagName: 'span',
            className: 'app-emotion-slider-container',
            ui: {
                button: 'button',
                durationIndicator: '.app-duration-indicator',
                magnitudeIndicator: '.app-magnitude-indicator',
                durationBar: '.app-duration-bar',
                magnitudeBar: '.app-magnitude-bar'
            },
            events: {
                'click @ui.button': 'onClick'
            },
            /**
             * Store config, hook on to TouchButton behavior events
             * @param options
             */
            initialize: function (options) {
                this.config = JSON.parse(JSON.stringify(options.config));
                // set current parameter values to default
                this.config.duration.current = this.config.duration.default;
                this.config.magnitude.current = this.config.magnitude.default;
            },
            /**
             * Handles regular click event on a desktop
             */
            onClick: function () {
                if (!this.touch) {
                    var config = this.options.getSliderValues();
                    this.setEmotion(config.duration, config.magnitude);
                    api.loginfo('[CLICK ACTION][EMOTION] '+this.model.get('name'));
                }

                this.touch = false;
            },
            setEmotion: function (duration, magnitude) {
                if (typeof duration == 'undefined')
                    duration = this.config.duration.current.toFixed(2);
                if (typeof magnitude == 'undefined')
                    magnitude = this.config.magnitude.current.toFixed(2);
                if (duration > 0)
                    api.setEmotion(this.model.get('name'), parseFloat(magnitude), parseFloat(duration));
            }
        });
    });

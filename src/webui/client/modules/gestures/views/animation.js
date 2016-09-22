define(["marionette", "./templates/animation.tpl", 'lib/api'],
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
            },
            onClick: function () {
                var config = this.options.getSliderValues();
                this.setGesture(config.speed, config.magnitude);
                api.loginfo('[CLICK ACTION][GESTURE] ' + this.model.get('name'));
            },
            setGesture: function (speed, magnitude) {
                if (typeof speed == 'undefined')
                    speed = this.config.speed.current.toFixed(2);
                if (typeof magnitude == 'undefined')
                    magnitude = this.config.magnitude.current.toFixed(2);
                if (speed > 0)
                    api.setGesture(this.model.get('name'), 1, parseFloat(speed), parseFloat(magnitude));
            }
        });
    });

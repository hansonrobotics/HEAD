define(['application', 'lib/api', 'lib/utilities'], function (App, api, utilities) {
    return Backbone.Model.extend({
        initialize: function () {
            // not selected by default
            this.set('selected', false);

            // not selected by default
            this.set('selected', false);

            // send motor command when updating value
            this.on('change:value', this.checkUpdatedValue);
        },
        selected: function (selected) {
            if (typeof selected == 'undefined') {
                // return status if arg undefined
                return this.get('selected');
            } else {
                // set status otherwise
                this.set('selected', !!selected);
            }
        },
        checkUpdatedValue: function () {
            if (this.previous('value') != this.get('value'))
            // If multiple motor values are changed the undescore fuction by pas limit of  30msg/s
                api._sendMotorCommand(this.toJSON(), this.get('value'));
        },
        getDegrees: function (attribute) {
            return Math.round(utilities.radToDeg(this.get(attribute))) || 0;
        },
        setDegrees: function (attribute, value, options) {
            return this.set(attribute, utilities.degToRad(value), options);
        },
        // Sets value 0-1 mapped to min and max.
        setRelativeVal: function (attribute, value) {
            var min = this.get('min');
            var max = this.get('max');
            var v = (min + (max - min) * value);
            this.set(attribute, v);
        },
        getRelativeVal: function (attribute) {
            var min = this.get('min');
            var max = this.get('max');
            var v = this.get(attribute);
            return Math.min(1, Math.max(0, (v - min) / (max - min)));
        }
    });
});


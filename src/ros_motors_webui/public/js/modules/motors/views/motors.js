define(["marionette", "./motor", 'jquery-ui'],
    function (Marionette, motorView) {
        return Marionette.CollectionView.extend({
            childView: motorView,
            /**
             * Pass label of the motor group to motor views when a new group appears
             *
             * @param model
             * @returns {{motorGroupLabel: *}}
             */
            childViewOptions: function (model) {
                var motorGroup = model.get('group');
                if (motorGroup && (typeof this.lastMotorGroup == 'undefined'
                    || this.lastMotorGroup != motorGroup)
                ) {
                    this.lastMotorGroup = motorGroup;
                } else {
                    motorGroup = null;
                }

                return {
                    motorGroupLabel: motorGroup
                }
            }
        });
    });

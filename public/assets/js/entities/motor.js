define(['app', 'main/lib/api', 'main/lib/utilities'], function (UI, api, utilities) {
    UI.module('Entities', function (Entities, UI, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            // converts given attribute value to radians
            getMinDeg: function () {
                return Math.ceil(utilities.radToDeg(this.get('min')));
            },
            getMaxDeg: function () {
                return Math.floor(utilities.radToDeg(this.get('max')));
            },
            getDefaultDeg: function () {
                return Math.round(utilities.radToDeg(this.get('default')));
            },
            setValueDeg: function (deg) {
                this.set('value', utilities.degToRad(deg));

                if (this.get('name') == "neck_roll") {
                    api.pointHead({roll: utilities.degToRad(this.get('value'))});
                } else {
                    api.sendMotorCommand(this.toJSON(), utilities.degToRad(this.get('value')));
                }
            }
        })
        ;
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor
        });
    })
    ;
})
;

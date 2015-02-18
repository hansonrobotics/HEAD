define(['app'], function (UI) {
    UI.module('Entities', function (Entities, UI, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({});
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor
        });
    });
});

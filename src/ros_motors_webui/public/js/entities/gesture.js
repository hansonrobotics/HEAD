define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Gesture = Backbone.Model.extend();
        Entities.GestureCollection = Backbone.Collection.extend({
            model: Entities.Gesture,
            fetch: function () {
                var self = this;

                api.getAvailableGestures(function (gestures) {
                    _.each(gestures, function (name) {
                        self.add({name: name});
                    });
                });
            }
        });
    });
});

define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Cycle = Backbone.Model.extend();
        Entities.CycleCollection = Backbone.Collection.extend({
            model: Entities.Cycle,
            fetch: function () {
                var self = this;

                api.getAvailableSomaStates(function (somaState) {
                    _.each(somaState, function (name) {
                        self.add({name: name});
                    });
                });
            }
        });
    });
});

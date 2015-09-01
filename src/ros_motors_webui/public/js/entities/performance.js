define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Performance = Backbone.Model.extend();
        Entities.PerformanceCollection = Backbone.Collection.extend({
            model: Entities.Performance,
            added: false,
            fetch: function () {
                var self = this;
                api.getAvailableScripts(function (scripts) {
                    if (!self.added){
                        self.added = true;
                        _.each(scripts, function (name) {
                            console.log(name)
                            self.add({name: name});
                        });
                    }
                });
            }
        });
    });
});

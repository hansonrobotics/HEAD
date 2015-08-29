define(['application', './node'], function (App) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Performance = Backbone.Model.extend({
            initialize: function () {
                this.set('nodes', new App.Performances.Entities.NodeCollection());
            }
        });
        Entities.PerformanceCollection = Backbone.Collection.extend({
            model: Entities.Performance,
            testFetch: function () {
                this.add(new Entities.Performance({name: "Greet"}));
            }
        });
    });
});

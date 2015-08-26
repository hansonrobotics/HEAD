define(['application'], function (App) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Timeline = Backbone.Model.extend();
        Entities.TimelineCollection = Backbone.Collection.extend({
            model: Entities.Timeline,
            testFetch: function () {
                this.add(new Entities.Timeline({name: "Greet"}));
            }
        });
    });
});

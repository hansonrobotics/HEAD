define(['application'], function (App) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Node = Backbone.Model.extend();
        Entities.NodeCollection = Backbone.Collection.extend({
            model: Entities.Node
        });
    });
});

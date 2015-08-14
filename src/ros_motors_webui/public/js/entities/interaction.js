define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Message = Backbone.Model.extend();
        Entities.MessageCollection = Backbone.Collection.extend({
            model: Entities.Message
        });
    });
});

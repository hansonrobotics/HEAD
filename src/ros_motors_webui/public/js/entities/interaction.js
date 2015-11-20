define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Message = Backbone.Model.extend({
            initialize: function () {
                this.set('time_created', new Date().getTime());
            }
        });
        Entities.MessageCollection = Backbone.Collection.extend({
            model: Entities.Message,
            comparator: function(item) {
                // two second priority for user messages
                return [item.get('time_created') / 3000, item.get('author') == 'Me' ? 1 : 2];
            }
        });
    });
});

define(['application'], function (App) {
    App.module('Performances.Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Node = Backbone.Model.extend({
            toJSON: function () {
                var json = Backbone.Model.prototype.toJSON.call(this);
                if (this.get('el')) delete json['el'];

                return json;
            }
        });
        Entities.NodeCollection = Backbone.Collection.extend({
            model: Entities.Node,
            comparator: 'offset'
        });
    });
});

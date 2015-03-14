define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Animation = Backbone.Model.extend({});
        Entities.AnimationsCollection = Backbone.Collection.extend({
            model: Entities.Expression,
            fetch: function() {
                var self = this;
                api.getAnimationsFromFile(function (animations) {
                    _.each(animations, function (animation) {
                        _.each(animation, function (frames, name) {
                            self.add(new Backbone.Model({
                                name: name,
                                frames_collection: new Backbone.Collection(frames)
                            }));
                        });
                    });
                });
            },
            sync: function () {
                self.each(function (animation) {
                    //console.log
                });
            }
        });
    });
});

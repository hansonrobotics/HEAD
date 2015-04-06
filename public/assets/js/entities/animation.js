define(['application', 'lib/api'], function (App, api) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.FramesCollection = Backbone.Collection.extend({
            comparator: 'order_no'
        });
        Entities.Animation = Backbone.Model.extend({});
        Entities.AnimationsCollection = Backbone.Collection.extend({
            model: Entities.Expression,
            fetch: function () {
                var self = this;
                api.getAnimations(function (animations) {
                    _.each(animations, function (animation) {
                        _.each(animation, function (frames, name) {
                            self.add(new Backbone.Model({
                                name: name,
                                frames_collection: new Entities.FramesCollection(frames)
                            }));
                        });
                    });
                });
            },
            sync: function () {
                var animations = [];

                this.each(function (model) {
                    var animation = {};
                    var name = model.get('name');

                    animation[name] = model.get('frames_collection').toJSON();
                    animation[name] = _.sortBy(animation[name], function (frame) {
                        return frame['order_no'];
                    });

                    _.each(animation[name], function (frame) {
                        delete frame['order_no'];
                    });

                    animations.push(animation);
                });

                api.updateAnimations(animations);
            }
        });
    });
});

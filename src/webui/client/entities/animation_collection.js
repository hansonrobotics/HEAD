define(['application', 'lib/api', './frame_collection'], function (App, api, FrameCollection) {
    return Backbone.Collection.extend({
        fetch: function () {
            var self = this;
            api.getAnimations(function (animations) {
                _.each(animations, function (animation) {
                    self.add({
                        name: animation.name,
                        frames_collection: new FrameCollection(animation.frames)
                    });
                });
            });
        },
        sync: function (successCallback, errorCallback) {
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

            api.updateAnimations(animations, successCallback, errorCallback);
        }
    });
});

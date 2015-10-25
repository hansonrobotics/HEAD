define(['backbone', 'lib/api', 'underscore', 'lib/utilities'], function (Backbone, api, _, utilities) {
    var face = Backbone.Model.extend({
        getThumbnailUrl: function () {
            return '/public/faces/Face' + this.id + '.jpg';
        }
    });

    return Backbone.Collection.extend({
        model: face,
        subscribe: function () {
            var self = this,
                callback = function (response) {
                    this.add(response.faces);
                    this.each(function (model) {
                        if (model && !_.findWhere(response.faces, {id: model.get('id')}))
                            self.remove(model);
                    });

                    self.trigger('change');
                };

            this.unsubscribe();
            this.subscribeCallback = utilities.limitCallRate(1, function () {
                callback.apply(self, arguments);
            });

            api.topics.face_locations.subscribe(this.subscribeCallback);
        },
        unsubscribe: function () {
            if (this.subscribeCallback) {
                api.topics.face_locations.unsubscribe(this.subscribeCallback);
                api.topics.face_locations.removeAllListeners();
            }
        }
    });
});
